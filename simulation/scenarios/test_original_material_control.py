"""The material control changes appearance only, using retained source images/UVs."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from PIL import Image

SCENARIOS=Path(__file__).resolve().parent
BASE=SCENARIOS/'indoor_tiny_house_living_room.xml'
VARIANT=SCENARIOS/'indoor_tiny_house_living_room_original_material_control.xml'
ASSETS=SCENARIOS.parent/'assets/aws_robomaker_small_house'
ITEMS=[('sofa','SofaC','6b58288bc4e4_aws_SofaC_01.png'),
       ('coffee_table','CoffeeTable','fc720fe6a63a_aws_CoffeeTable_01.png')]


def structure(element):
    return element.tag,sorted(element.attrib.items()),(element.text or '').strip(),[structure(child) for child in element]


class OriginalMaterialControlTests(unittest.TestCase):
    def test_only_two_textures_materials_and_geom_appearance_change(self):
        base=ET.parse(BASE).getroot();variant=ET.parse(VARIANT).getroot()
        asset=variant.find('asset')
        for name,_,filename in ITEMS:
            texture=asset.find(f'texture[@name="original_{name}_texture"]')
            material=asset.find(f'material[@name="original_{name}_material"]')
            self.assertEqual(texture.attrib,dict(name=f'original_{name}_texture',type='2d',
                file='../assets/aws_robomaker_small_house/converted/textures/'+filename))
            self.assertEqual(material.attrib,dict(name=f'original_{name}_material',texture=f'original_{name}_texture',texuniform='false'))
            geom=variant.find(f'worldbody/geom[@name="env_mesh_{name}"]')
            original=base.find(f'worldbody/geom[@name="env_mesh_{name}"]')
            self.assertEqual(geom.get('material'),f'original_{name}_material')
            self.assertEqual(geom.get('rgba'),'1 1 1 1')
            del geom.attrib['material'];geom.set('rgba',original.get('rgba'))
            asset.remove(texture);asset.remove(material)
        self.assertEqual(structure(variant),structure(base))

    def test_original_dae_image_links_rgb_pixels_and_obj_uvs_are_retained(self):
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        base=ET.parse(BASE).getroot()
        for name,source_name,filename in ITEMS:
            dae=ASSETS/f'source/models/aws_robomaker_residential_{source_name}_01/meshes/aws_{source_name}_01_visual.DAE'
            source=ET.parse(dae).getroot()
            images=source.findall('c:library_images/c:image/c:init_from',ns)
            self.assertEqual(len(images),1)
            self.assertEqual(Path(images[0].text).name,f'aws_{source_name}_01.png')
            with Image.open(dae.parent/images[0].text) as original,Image.open(ASSETS/'converted/textures'/filename) as converted:
                self.assertEqual(original.size,converted.size)
                self.assertEqual(original.convert('RGB').tobytes(),converted.convert('RGB').tobytes())
            self.assertTrue(source.findall('.//c:input[@semantic="TEXCOORD"]',ns))
            mesh=base.find(f'asset/mesh[@name="env_mesh_{name}_asset"]')
            lines=(SCENARIOS/mesh.get('file')).read_text().splitlines()
            uv_count=sum(line.startswith('vt ') for line in lines)
            self.assertGreater(uv_count,0)
            for line in lines:
                if line.startswith('f '):
                    self.assertTrue(all(1<=int(vertex.split('/')[1])<=uv_count for vertex in line.split()[1:]))


if __name__=='__main__':unittest.main()
