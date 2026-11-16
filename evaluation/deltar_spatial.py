"""Diagnostic calibrated association; no production model or checkpoint changes.

Feature cell (r,c) represents input-image center ((r+.5)*stride,
(c+.5)*stride). A zone owns centers in its half-open [y0,y1)x[x0,x1)
rectangle. Queries and residual outputs use precisely the same cells. Overlap
outputs are averaged, not summed. No padded or out-of-image query is created.
"""
import torch
from src.models.fusion import TransformerFusion


class CalibratedTransformerFusion(TransformerFusion):
    def forward(self, x, feat1, **kwargs):
        B,D,H,W=x.shape
        rect=kwargs['rect_data'];valid=kwargs['mask'].bool()
        if rect.shape != (B,feat1.shape[1],4) or valid.shape != feat1.shape[:2]:
            raise ValueError('rectangle/mask shape must match batch and zone IDs')
        # Inspect only valid geometry: invalid boxes/histograms have no influence.
        boxes=rect[valid]
        if not torch.isfinite(boxes).all() or ((boxes[:,2:] - boxes[:,:2]) <= 0).any():
            raise ValueError('valid zone rectangles must be finite with positive area')
        oy=ox=0
        if H < self.max_resolution[0]:
            oy=int(torch.randint(0,self.max_resolution[0]-H+1,[1]))
        if W < self.max_resolution[1]:
            ox=int(torch.randint(0,self.max_resolution[1]-W+1,[1]))
        pos=self.positional_encodings.view(*self.max_resolution,-1)[oy:oy+H,ox:ox+W].permute(2,0,1)
        embeddings=x+pos
        feat0=embeddings.flatten(2).transpose(1,2)
        hist=feat1+self.positional_encodings2[None,None]
        ys=(torch.arange(H,device=x.device)+.5)*self.conv_patch_size
        xs=(torch.arange(W,device=x.device)+.5)*self.conv_patch_size
        membership=[]
        for b in range(B):
            zones=[]
            for z in torch.nonzero(valid[b],as_tuple=False).flatten().tolist():
                y0,x0,y1,x1=rect[b,z].to(x.device)
                cells=((ys[:,None]>=y0)&(ys[:,None]<y1)&(xs[None,:]>=x0)&(xs[None,:]<x1)).flatten().nonzero(as_tuple=False).flatten()
                if cells.numel():zones.append((z,cells))
            membership.append(zones)
        original=embeddings.flatten(2).transpose(1,2)
        image_layer_seen=False
        for layer,name in zip(self.layers,self.layer_names):
            if name=='image':
                feat0=layer(feat0,(H,W))
                image_layer_seen=True
            elif name=='hist2image':
                updates=[]
                for b,zones in enumerate(membership):
                    update=torch.zeros_like(original[b]);count=x.new_zeros((H*W,1))
                    for z,cells in zones:
                        # Legacy shares embedding storage until the first image
                        # layer; later cross-attention queries retain that snapshot.
                        out=layer(original[b:b+1,cells],hist[b:b+1,z])[0]
                        update=update.index_add(0,cells,out)
                        count=count.index_add(0,cells,x.new_ones((cells.numel(),1)))
                    updates.append(update/count.clamp_min(1))
                feat0=feat0+torch.stack(updates)
                if not image_layer_seen:
                    original=feat0
        return feat0.transpose(1,2).reshape(B,D,H,W)
