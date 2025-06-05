import os
import sys
import xml.etree.ElementTree as ET

def main():
    if len(sys.argv) < 2:
        return
    
    model_dir = sys.argv[1]
    base_name = os.path.basename(os.path.normpath(model_dir))
    dae_path = os.path.join(model_dir, 'meshes', base_name + '.dae')
    
    if not os.path.exists(dae_path):
        return
    
    tree = ET.parse(dae_path)
    root = tree.getroot()
    
    # Extract namespace
    ns_uri = ''
    if '}' in root.tag:
        ns_uri = root.tag.split('}', 1)[0][1:]
    
    # Register default namespace to avoid ns0: prefixes
    if ns_uri:
        ET.register_namespace('', ns_uri)
    
    dae_tag = lambda tag: f'{{{ns_uri}}}{tag}' if ns_uri else tag
    
    parent_map = {child: parent for parent in root.iter() for child in parent}
    
    textures = [elem for elem in root.iter(dae_tag('texture'))]
    
    for texture in textures:
        parent = parent_map.get(texture)
        if parent is None:
            continue
        grandparent = parent_map.get(parent)
        if grandparent is None:
            continue
        
        ambient_exists = any(child.tag == dae_tag('ambient') for child in grandparent)
        if ambient_exists:
            continue
        
        ambient = ET.Element(dae_tag('ambient'))
        color = ET.SubElement(ambient, dae_tag('color'))
        color.set('sid', 'ambient')
        color.text = '0.9 0.9 0.9 1'
        grandparent.insert(0, ambient)
    
    tree.write(dae_path, encoding='utf-8', xml_declaration=True)

if __name__ == '__main__':
    main()