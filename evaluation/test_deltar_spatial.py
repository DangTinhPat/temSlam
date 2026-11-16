"""CPU contract tests; run with the DELTAR environment."""
import copy
from pathlib import Path
import sys
import unittest
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/model/deltar'))
_saved = sys.argv
sys.argv = ['test']
from src.config import args
sys.argv = _saved
from src.models.fusion import TransformerFusion
from src.utils.dataloader import patch_info_from_rect_data
from deltar_spatial import CalibratedTransformerFusion


def fixture(stride=4, batch=1):
    rect = torch.tensor([[r*2*stride,c*2*stride,(r+1)*2*stride,(c+1)*2*stride]
                         for r in range(8) for c in range(8)], dtype=torch.float32).repeat(batch,1,1)
    return rect, torch.ones(batch,64,dtype=torch.bool)


def module(cls, stride=4):
    args.attention_layer=['hist2image','image','hist2image','image']
    return cls(8,(480//stride,640//stride),num_heads=2).eval()


def extra(rect, mask):
    from torch.utils.data._utils.collate import default_collate
    return dict(rect_data=rect,mask=mask,patch_info=default_collate([patch_info_from_rect_data(r) for r in rect]))


class SpatialTests(unittest.TestCase):
    def test_regular_grid_matches_legacy_all_scales_strict_weights(self):
        torch.manual_seed(12)
        for stride in (4,8,16):
            old=module(TransformerFusion,stride);new=module(CalibratedTransformerFusion,stride)
            new.load_state_dict(old.state_dict(),strict=True)
            rect,mask=fixture(stride)
            x=torch.randn(1,8,480//stride,640//stride)
            hist=torch.randn(1,64,16,8)
            with torch.no_grad():
                # Avoid randomized crop offsets: full architecture resolution.
                torch.testing.assert_close(new(x,hist,**extra(rect,mask)),old(x,hist,**extra(rect,mask)),atol=1e-5,rtol=1e-5)

    def test_invalid_outlier_and_hist_cannot_move_valid_zone(self):
        model=module(CalibratedTransformerFusion);model.layer_names=['hist2image'];model.layers=torch.nn.ModuleList([model.layers[0]])
        rect,mask=fixture();mask[:,33]=False
        x=torch.randn(1,8,16,16);hist=torch.randn(1,64,16,8)
        changed=rect.clone();changed[:,33]=torch.tensor([-1e8,-1e8,1e8,1e8])
        hist2=hist.clone();hist2[:,33]=float('nan')
        torch.manual_seed(1);before=model(x,hist,rect_data=rect,mask=mask)
        torch.manual_seed(1);after=model(x,hist2,rect_data=changed,mask=mask)
        torch.testing.assert_close(before,after,atol=0,rtol=0)

class SourceValue(torch.nn.Module):
    def forward(self,x,source):
        return source.mean(1,keepdim=True).expand_as(x).clone()


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.model=module(CalibratedTransformerFusion,16)
        self.model.layer_names=['hist2image']
        self.model.layers=torch.nn.ModuleList([SourceValue()])
        with torch.no_grad():
            self.model.positional_encodings.zero_();self.model.positional_encodings2.zero_()

    def call(self,x,hist,rect,mask):
        torch.manual_seed(1)
        return self.model(x,hist,rect_data=rect,mask=mask)

    def test_irregular_overlap_oob_missing_outer_identity_and_gradient(self):
        x=torch.zeros(1,8,4,4,requires_grad=True)
        hist=torch.zeros(1,64,16,8,requires_grad=True)
        values=torch.zeros_like(hist);values[:,9]=2;values[:,55]=6
        rect=torch.full((1,64,4),float('nan'));mask=torch.zeros(1,64,dtype=torch.bool)
        mask[:,[9,55]]=True
        rect[:,9]=torch.tensor([-100.,-100.,32.,32.])
        rect[:,55]=torch.tensor([16.,16.,48.,48.])
        out=self.call(x,hist+values,rect,mask)
        expected=torch.zeros_like(out);expected[:,:,:2,:2]=2;expected[:,:,1:3,1:3]=6;expected[:,:,1,1]=4
        torch.testing.assert_close(out,expected)
        out.sum().backward()
        self.assertTrue(torch.isfinite(x.grad).all());self.assertTrue(torch.isfinite(hist.grad).all())
        self.assertEqual(hist.grad[:,0].abs().sum().item(),0)

    def test_legacy_invalid_outlier_regression_red_and_green(self):
        rect,mask=fixture(16);mask[:,33]=False
        changed=rect.clone();changed[:,33,1::2]-=64
        hist=torch.arange(64,dtype=torch.float32)[None,:,None,None].expand(1,64,16,8)
        x=torch.zeros(1,8,30,40)
        old=module(TransformerFusion,16);new=module(CalibratedTransformerFusion,16)
        for model in (old,new):
            model.layer_names=['hist2image'];model.layers=torch.nn.ModuleList([SourceValue()])
            with torch.no_grad():model.positional_encodings.zero_();model.positional_encodings2.zero_()
        with torch.no_grad():
            baseline=old(x,hist,**extra(rect,mask));polluted=old(x,hist,**extra(changed,mask))
            self.assertGreater((baseline-polluted).abs().max().item(),1)
            torch.testing.assert_close(new(x,hist,**extra(rect,mask)),new(x,hist,**extra(changed,mask)),atol=0,rtol=0)

    def test_all_invalid_empty_oob_and_valid_malformed(self):
        x=torch.randn(1,8,4,4);hist=torch.full((1,64,16,8),float('nan'))
        rect=torch.full((1,64,4),float('nan'));mask=torch.zeros(1,64,dtype=torch.bool)
        torch.testing.assert_close(self.call(x,hist,rect,mask),x)
        mask[:,63]=True;hist[:,63]=1;rect[:,63]=torch.tensor([100.,100.,120.,120.])
        torch.testing.assert_close(self.call(x,hist,rect,mask),x)
        rect[:,63]=torch.tensor([0.,0.,1.,1.])
        torch.testing.assert_close(self.call(x,hist,rect,mask),x)
        rect[:,63]=torch.tensor([1.,2.,1.,4.])
        with self.assertRaisesRegex(ValueError,'positive area'):self.call(x,hist,rect,mask)
        rect[:,63]=float('nan')
        with self.assertRaisesRegex(ValueError,'finite'):self.call(x,hist,rect,mask)

    def test_all_invalid_matches_actual_image_only_layers(self):
        model=module(CalibratedTransformerFusion,16)
        x=torch.randn(1,8,30,40);hist=torch.full((1,64,16,8),float('nan'))
        rect=torch.full((1,64,4),float('nan'));mask=torch.zeros(1,64,dtype=torch.bool)
        expected=(x+model.positional_encodings.reshape(30,40,8).permute(2,0,1)).flatten(2).transpose(1,2)
        with torch.no_grad():
            for layer,name in zip(model.layers,model.layer_names):
                if name=='image':expected=layer(expected,(30,40))
            actual=model(x,hist,rect_data=rect,mask=mask)
        torch.testing.assert_close(actual,expected.transpose(1,2).reshape_as(x),atol=0,rtol=0)

    def test_batch_independence_and_zone_permutation(self):
        x=torch.zeros(2,8,4,4);hist=torch.randn(2,64,16,8)
        rect,mask=fixture(16,2);mask[:]=False;mask[:,[9,55]]=True
        rect[:,9]=torch.tensor([0.,0.,32.,32.]);rect[:,55]=torch.tensor([16.,16.,48.,48.])
        together=self.call(x,hist,rect,mask)
        alone=self.call(x[:1],hist[:1],rect[:1],mask[:1])
        torch.testing.assert_close(together[:1],alone,atol=0,rtol=0)
        order=torch.arange(63,-1,-1)
        torch.testing.assert_close(together,self.call(x,hist[:,order],rect[:,order],mask[:,order]))


if __name__=='__main__': unittest.main()
