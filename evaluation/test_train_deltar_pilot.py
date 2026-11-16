"""CPU checks for the measured-H5, tail-only training boundary."""
import tempfile
import hashlib
from pathlib import Path
import unittest
import torch
import h5py
import numpy as np
from train_deltar_pilot import build_model, freeze_tail, state_hashes, supervised_loss, load_measurements, load_target, training_order, check_limits


class PilotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=build_model()

    def test_exact_optimizer_scope_bn_frozen_gradient_and_checkpoint(self):
        model=self.model;parameters=freeze_tail(model)
        self.assertEqual(sum(p.numel() for p in parameters),431040)
        allowed=('decoder.up4.','decoder.conv0.','depth_head.','conv_out.')
        self.assertEqual({id(p) for p in parameters},{id(p) for n,p in model.named_parameters() if n.startswith(allowed)})
        self.assertTrue(all(not module.training for module in model.modules() if isinstance(module,torch.nn.modules.batchnorm._BatchNorm)))
        before=state_hashes(model)
        optimizer=torch.optim.AdamW(parameters,lr=1e-5,weight_decay=.01)
        # Exercise the actual four trainable modules on small feature fixtures;
        # upstream calibrated RGB/ToF inference is covered separately.
        features=model.decoder.up4(torch.randn(1,64,4,4),torch.randn(1,16,8,8))
        decoded=model.decoder.conv0(features)
        widths,attention=model.depth_head(decoded)
        probability=model.conv_out(attention)
        edges=torch.cumsum(torch.nn.functional.pad(widths*(10-.001),(1,0),value=.001),dim=1)
        centers=(edges[:,:-1]+edges[:,1:])/2
        prediction=(probability*centers[:,:,None,None]).sum(1,keepdim=True)
        target=torch.ones(1,1,8,8);target[:,:,0,:]=0;target[:,:,1,0]=float('nan')
        loss=supervised_loss(prediction,target)
        self.assertTrue(torch.isfinite(loss));loss.backward()
        self.assertTrue(all(p.grad is None for p in model.parameters() if not p.requires_grad))
        self.assertTrue(all(p.grad is not None and torch.isfinite(p.grad).all() for p in parameters))
        self.assertGreater(sum(p.grad.abs().sum().item() for p in parameters),0)
        optimizer.step();after=state_hashes(model)
        changed={key for key in before if before[key]!=after[key]}
        self.assertTrue(changed);self.assertTrue(all(key.startswith(allowed) and key in dict(model.named_parameters()) for key in changed))
        self.assertTrue(all(before[key]==after[key] for key in before if key not in dict(model.named_parameters()) or not key.startswith(allowed)))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'candidate.pt';torch.save(model.state_dict(),path)
            restored=build_model(path)
            self.assertEqual(after,state_hashes(restored))

    def test_separate_label_never_enters_measured_input_and_placeholder_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'input.h5';label=Path(tmp)/'target.npy'
            with h5py.File(source,'w') as h5:
                h5['rgb']=np.full((480,640,3),127,np.uint8);h5['depth']=np.zeros((480,640),np.float32)
                h5['hist_data']=np.tile([1.2,.005],(64,1)).astype(np.float32)
                h5['mask']=np.ones(64,bool)
                h5['fr']=np.array([[r*60,c*80,(r+1)*60,(c+1)*80] for r in range(8) for c in range(8)],np.float32)
            source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
            torch.manual_seed(2026);first=load_measurements(source)
            np.save(label,np.ones((480,640),np.float32));a=load_target(label)
            np.save(label,np.full((480,640),3,np.float32));b=load_target(label)
            torch.manual_seed(2026);second=load_measurements(source)
            self.assertEqual(source_hash,hashlib.sha256(source.read_bytes()).hexdigest())
            def compare(left,right):
                if isinstance(left,dict):
                    self.assertEqual(set(left),set(right))
                    for key in left:compare(left[key],right[key])
                else:torch.testing.assert_close(left,right,atol=0,rtol=0)
            compare(first,second)
            self.assertEqual(set(first),{'rgb','additional'})
            self.assertEqual(set(first['additional']),{'hist_data','rect_data','mask','patch_info'})
            self.assertFalse(torch.equal(a,b));torch.testing.assert_close(first['rgb'],second['rgb'],atol=0,rtol=0)
            torch.testing.assert_close(first['additional']['hist_data'],second['additional']['hist_data'],atol=0,rtol=0)
            self.assertEqual(first['rgb'].shape,(1,3,480,640));self.assertEqual(first['additional']['hist_data'].shape,(1,64,16))
            with h5py.File(source,'r+') as h5:h5['depth'][:]=1
            with self.assertRaisesRegex(ValueError,'placeholder'):load_measurements(source)
            np.save(label,np.ones((240,320),np.float32))
            with self.assertRaisesRegex(ValueError,'480'):load_target(label)

    def test_split_disjointness_and_exact_original_line_hash(self):
        import copy
        from prepare_deltar_pilot import validate_manifest,verified_line,verify_frozen_sources
        samples=[dict(sample_id=f'train{i}',split='train',capture_file='a.jsonl',capture_line_1based=i+1) for i in range(100)]
        samples.extend(dict(sample_id=f'val{i}',split='validation',capture_file='b.jsonl',capture_line_1based=i+1) for i in range(48))
        validate_manifest(dict(samples=samples))
        overlap=copy.deepcopy(samples);overlap[-1].update(capture_file='a.jsonl',capture_line_1based=1)
        with self.assertRaisesRegex(ValueError,'overlap'):validate_manifest(dict(samples=overlap))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'capture.jsonl';line=b'{"t":1.0}\n';path.write_bytes(line)
            expected=hashlib.sha256(line).hexdigest()
            verify_frozen_sources({'provenance':{'source_file_sha256':{str(path):expected}}})
            with self.assertRaisesRegex(ValueError,'Frozen source'):
                verify_frozen_sources({'provenance':{'source_file_sha256':{str(path):'0'*64}}})
            self.assertEqual(verified_line(path,1,expected),{'t':1.})
            path.write_bytes(b'{"t":2.0}\n')
            with self.assertRaisesRegex(ValueError,'hash'):verified_line(path,1,expected)

    def test_fixed_train_sampler_and_hard_budget_boundaries(self):
        samples=[dict(sample_id=f'train{i}',split='train') for i in range(161)]+[dict(sample_id=f'val{i}',split='validation') for i in range(48)]
        order=training_order(samples)
        self.assertEqual(len(order),100);self.assertEqual(len(set(order)),100)
        self.assertTrue(all(samples[i]['split']=='train' for i in order))
        self.assertEqual(order,training_order(samples))
        check_limits(599.,int(9.4*2**30))
        with self.assertRaisesRegex(RuntimeError,'600'):check_limits(600.1,1)
        with self.assertRaisesRegex(RuntimeError,'9.5'):check_limits(1,10*2**30)

    def test_loss_rejects_insufficient_and_nonfinite_prediction(self):
        with self.assertRaisesRegex(ValueError,'two'):
            supervised_loss(torch.ones(1,1,2,2),torch.zeros(1,1,2,2))
        with self.assertRaisesRegex(ValueError,'prediction'):
            supervised_loss(torch.full((1,1,2,2),float('nan')),torch.ones(1,1,2,2))

if __name__=='__main__':unittest.main()
