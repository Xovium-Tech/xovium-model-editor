"""Portable model exports. Geometry authoring is separate from runtime sensor integration."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET


_TYPES = {'airspeed': 'air_speed', 'gps': 'navsat', 'barometer': 'air_pressure',
          'imu': 'imu', 'magnetometer': 'magnetometer', 'camera': 'camera',
          'depth_camera': 'depth_camera', 'lidar': 'gpu_lidar', 'contact': 'contact'}
_SYSTEMS = {'airspeed': 'AirSpeed', 'gps': 'NavSat', 'barometer': 'AirPressure',
            'imu': 'Imu', 'magnetometer': 'Magnetometer', 'camera': 'Sensors',
            'depth_camera': 'Sensors', 'lidar': 'Sensors', 'contact': 'Contact'}


def _safe(value):
    return re.sub(r'[^A-Za-z0-9_]', '_', str(value)) or 'model'


def _usd_name(value):
    value = _safe(value)
    return '_' + value if value[0].isdigit() else value


def _nums(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def _array(values):
    return '(' + ', '.join(format(float(v), '.12g') for v in values) + ')'


def _asset(workspace, relative):
    path = (workspace / relative).resolve()
    if not path.is_relative_to(workspace.resolve()) or not path.is_file():
        raise ValueError('Missing or external workspace asset: ' + str(relative))
    return path


def _matrix(pose=None):
    x, y, z, r, p, h = pose or [0] * 6
    cr, sr, cp, sp, ch, sh = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(h), math.sin(h)
    return [[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr, x],
            [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr, y], [-sp, cp*sr, cp*cr, z], [0, 0, 0, 1]]


def _mul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _point(m, p):
    return [sum(m[i][k]*p[k] for k in range(3)) + m[i][3] for i in range(3)]


def _quat(m):
    trace = sum(m[i][i] for i in range(3))
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return [s / 4, (m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s]
    i = max(range(3), key=lambda axis: m[axis][axis]); j = (i+1) % 3; k = (i+2) % 3
    s = math.sqrt(max(0, 1 + m[i][i] - m[j][j] - m[k][k])) * 2
    q = [0., 0., 0., 0.]
    q[0] = (m[k][j]-m[j][k])/s; q[i+1] = s/4
    q[j+1] = (m[j][i]+m[i][j])/s; q[k+1] = (m[k][i]+m[i][k])/s
    return q


def _meshes(visual, workspace):
    geometry = visual['geometry']; kind = geometry['type']
    if kind == 'mesh':
        meshes = json.loads(_asset(workspace, geometry['path']).read_text())
        if isinstance(meshes, dict):
            meshes = meshes.get('meshes', meshes.get('parts'))
        if not isinstance(meshes, list):
            raise ValueError('Mesh asset must contain an array of mesh parts')
        meshes = copy.deepcopy(meshes)
    elif kind == 'box':
        sx, sy, sz = [v/2 for v in geometry['size']]
        vertices = [[x*sx, y*sy, z*sz] for x,y,z in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        meshes = [{'vertices':vertices,'faces':faces}]
    elif kind == 'cylinder':
        radius, length = geometry['radius'], geometry['length']
        vertices = [[radius*math.cos(i*math.tau/32), radius*math.sin(i*math.tau/32), z] for z in (-length/2,length/2) for i in range(32)]
        vertices += [[0,0,-length/2],[0,0,length/2]]; faces=[]
        for i in range(32):
            j=(i+1)%32; faces += [[i,j,j+32],[i,j+32,i+32],[64,j,i],[65,i+32,j+32]]
        meshes = [{'vertices':vertices,'faces':faces}]
    elif kind == 'sphere':
        radius=geometry['radius']; vertices=[]; faces=[]
        for j in range(17):
            p=math.pi*j/16
            for i in range(32): vertices.append([radius*math.sin(p)*math.cos(i*math.tau/32),radius*math.sin(p)*math.sin(i*math.tau/32),radius*math.cos(p)])
        for j in range(16):
            for i in range(32):
                a=j*32+i; b=j*32+(i+1)%32; faces += [[a,b,b+32],[a,b+32,a+32]]
        meshes = [{'vertices':vertices,'faces':faces}]
    else:
        raise ValueError('Unsupported geometry type: '+str(kind))
    for mesh in meshes:
        mesh.setdefault('uv', []); mesh.setdefault('color', visual.get('color', [.65,.65,.65,1])); mesh.setdefault('texture', '')
        if 'color' in visual:
            mesh['color'] = [a*b for a,b in zip(mesh['color'], visual['color'])] if kind == 'mesh' else visual['color']
        if mesh['texture']: _asset(workspace, mesh['texture'])
    return meshes


def _obj(meshes, path, texture_prefix='../'):
    path.parent.mkdir(parents=True, exist_ok=True)
    offset=1; uv_offset=1; lines=['mtllib '+path.with_suffix('.mtl').name]; materials=[]
    for index, mesh in enumerate(meshes):
        name='part_'+str(index); lines += ['o '+name, 'usemtl '+name]
        lines += ['v '+_nums(v) for v in mesh['vertices']]
        lines += ['vt '+_nums(v) for v in mesh['uv']]
        for face in mesh['faces']:
            lines.append('f '+' '.join(str(offset+j)+('/'+str(uv_offset+j) if mesh['uv'] else '') for j in face))
        offset += len(mesh['vertices']); uv_offset += len(mesh['uv'])
        materials += ['newmtl '+name, 'Kd '+_nums(mesh['color'][:3]), 'd '+str(mesh['color'][3])]
        if mesh['texture']: materials.append('map_Kd '+texture_prefix+mesh['texture'])
    path.write_text('\n'.join(lines)+'\n'); path.with_suffix('.mtl').write_text('\n'.join(materials)+'\n')


def _set(parent, path, value):
    for part in path.split('/'):
        child=parent.find(part)
        if child is None: child=ET.SubElement(parent,part)
        parent=child
    parent.text = str(value).lower() if isinstance(value, bool) else str(value)
    return parent


def _noise(parent, path, parameters):
    if isinstance(parameters, (float,int)):
        parameters={'stddev':parameters}
    if not isinstance(parameters, dict): return
    noise = parent
    for part in path.split('/'):
        found=noise.find(part)
        noise = found if found is not None else ET.SubElement(noise,part)
    noise.set('type', str(parameters.get('type',noise.get('type','gaussian'))))
    for key in ('mean','stddev','bias_mean','bias_stddev','dynamic_bias_stddev','dynamic_bias_correlation_time','precision'):
        if key in parameters: _set(noise,key,parameters[key])


def _parameter(p, *names, default=None):
    for name in names:
        value=p
        for part in name.split('.'):
            if not isinstance(value,dict) or part not in value: break
            value=value[part]
        else: return value
    return default


def _edit_imported_parameters(node, kind, parameters, baseline):
    """Change only catalog fields the user edited; leave unrelated raw XML intact."""
    paths = {
        'camera': {'width': ['camera/image/width'], 'height': ['camera/image/height'],
                   'horizontal_fov': ['camera/horizontal_fov'], 'near': ['camera/clip/near'], 'far': ['camera/clip/far']},
        'contact': {'collision': ['contact/collision']},
        'airspeed': {'noise': ['air_speed/pressure/noise']},
        'barometer': {'pressure_noise': ['air_pressure/pressure/noise']},
        'magnetometer': {'noise': ['magnetometer/'+axis+'/noise' for axis in 'xyz']},
        'imu': {name+'_noise': ['imu/'+field+'/'+axis+'/noise' for axis in 'xyz']
                for name,field in [('accel','linear_acceleration'),('gyro','angular_velocity')]},
        'gps': {name+'_noise': ['navsat/'+name+'_sensing/'+axis+'/noise' for axis in ('horizontal','vertical')]
                for name in ('position','velocity')},
    }
    if kind == 'depth_camera':
        paths[kind] = copy.deepcopy(paths['camera'])
        for field in ('near','far'): paths[kind][field].append('camera/depth_camera/clip/'+field)
    if kind == 'lidar':
        prefix = 'ray' if node.find('ray') is not None and node.find('lidar') is None else 'lidar'
        paths[kind] = {axis+'_'+field: [prefix+'/scan/'+axis+'/'+field]
                       for axis in ('horizontal','vertical') for field in ('samples','min_angle','max_angle')}
        paths[kind].update({field+'_range': [prefix+'/range/'+field] for field in ('min','max')})
    changed = {key for key,value in parameters.items() if key not in baseline or value != baseline[key]}
    if not changed.issubset(paths.get(kind,{})): return False
    for key in changed:
        for field in paths[kind][key]:
            if field.endswith('/noise'): _noise(node,field,parameters[key])
            else: _set(node,field,parameters[key])
    return True


def _sensor_xml(sensor, original=None):
    kind=sensor['type']; p=sensor.get('parameters',{})
    node=copy.deepcopy(original) if original is not None else ET.Element('sensor')
    node.set('name',sensor['name']); node.set('type',_TYPES[kind])
    pose=_set(node,'pose',_nums(sensor.get('pose',[0]*6))); pose.attrib.clear()
    if original is None: _set(node,'always_on',True)
    if original is None or sensor.get('rate_hz',0) != sensor.get('source_rate_hz'):
        _set(node,'update_rate',sensor.get('rate_hz',0))
    if sensor.get('topic'): _set(node,'topic',sensor['topic'])
    else:
        for topic in node.findall('topic'): node.remove(topic)
    if kind == 'contact' and sensor.get('topic'): _set(node,'contact/topic',sensor['topic'])
    aliases={'gps':'navsat','ray':'gpu_lidar','lidar':'gpu_lidar','gpu_ray':'gpu_lidar','depth':'depth_camera'}
    original_type=original.get('type') if original is not None else None
    if (original is not None and isinstance(sensor.get('source_parameters'),dict)
            and aliases.get(original_type,original_type)==_TYPES[kind]):
        if _edit_imported_parameters(node,kind,p,sensor['source_parameters']): return node
    if kind in ('camera','depth_camera'):
        _set(node,'camera/horizontal_fov',_parameter(p,'horizontal_fov','hfov','hfov_rad',default=1.0471975512))
        _set(node,'camera/image/width',_parameter(p,'width','image_width','image.width',default=640))
        _set(node,'camera/image/height',_parameter(p,'height','image_height','image.height',default=480))
        _set(node,'camera/image/format',_parameter(p,'format','image.format',default='R_FLOAT32' if kind=='depth_camera' else 'R8G8B8'))
        _set(node,'camera/clip/near',_parameter(p,'near','near_clip','clip.near',default=.1))
        _set(node,'camera/clip/far',_parameter(p,'far','far_clip','clip.far',default=100))
        if kind=='depth_camera':
            _set(node,'camera/depth_camera/clip/near',_parameter(p,'depth_near','near','near_clip','clip.near',default=.1))
            _set(node,'camera/depth_camera/clip/far',_parameter(p,'depth_far','far','far_clip','clip.far',default=100))
        if isinstance(p.get('distortion'),dict):
            for key,value in p['distortion'].items():
                if key in ('k1','k2','k3','p1','p2'): _set(node,'camera/distortion/'+key,value)
        if 'noise_stddev' in p: _set(node,'camera/noise/stddev',p['noise_stddev'])
    elif kind=='lidar':
        for legacy in node.findall('ray'):
            if node.find('lidar') is None: legacy.tag='lidar'
            else: node.remove(legacy)
        for axis,defaults in [('horizontal',(360,-math.pi,math.pi)),('vertical',(1,0,0))]:
            for field,default in zip(('samples','min_angle','max_angle'),defaults):
                value=_parameter(p,axis+'_'+field,axis+'.'+field,'scan.'+axis+'.'+field,default=default)
                _set(node,'lidar/scan/'+axis+'/'+field,value)
            _set(node,'lidar/scan/'+axis+'/resolution',_parameter(p,axis+'_resolution',axis+'.resolution',default=1))
        for field,default in [('min',.1),('max',30),('resolution',.01)]:
            _set(node,'lidar/range/'+field,_parameter(p,'range_'+field,'range.'+field,field+'_range',default=default))
        _set(node,'lidar/noise/type','gaussian'); _set(node,'lidar/noise/stddev',_parameter(p,'noise_stddev',default=0))
    elif kind=='contact':
        _set(node,'contact/collision',_parameter(p,'collision','collision_name',default='collision'))
        _set(node,'contact/topic',sensor.get('topic') or sensor['name']+'/contacts')
    elif kind in ('airspeed','barometer'):
        prefix=_TYPES[kind]
        _noise(node,prefix+'/pressure/noise',_parameter(p,'noise','pressure_noise',default={'stddev':p.get('noise_stddev',0)}))
        if kind=='barometer': _set(node,prefix+'/reference_altitude',p.get('reference_altitude',0))
    elif kind=='magnetometer':
        for axis in 'xyz': _noise(node,'magnetometer/'+axis+'/noise',_parameter(p,axis+'_noise','noise.'+axis,'noise',default={'stddev':p.get('noise_stddev',0)}))
    elif kind=='imu':
        for measurement,alias in [('angular_velocity','gyro'),('linear_acceleration','accel')]:
            for axis in 'xyz':
                _noise(node,'imu/'+measurement+'/'+axis+'/noise',_parameter(p,measurement+'.'+axis,'noise.'+measurement+'.'+axis,alias+'_noise',default={'stddev':p.get(alias+'_noise_stddev',0)}))
        if 'enable_orientation' in p: _set(node,'imu/enable_orientation',p['enable_orientation'])
    elif kind=='gps':
        for measurement,alias in [('position_sensing','position'),('velocity_sensing','velocity')]:
            for axis in ('horizontal','vertical'):
                _noise(node,'navsat/'+measurement+'/'+axis+'/noise',_parameter(p,measurement+'.'+axis,alias+'_'+axis+'_noise',alias+'_noise',default={'stddev':p.get(alias+'_'+axis+'_stddev',0)}))
    return node


def _copy_assets(workspace, output):
    assets=workspace/'assets'
    if not assets.exists(): return
    if assets.is_symlink(): raise ValueError('Workspace assets must not be a symbolic link')
    for source in assets.rglob('*'):
        if source.is_symlink(): raise ValueError('Workspace assets must not contain symbolic links')
        if source.is_file():
            target=output/'assets'/source.relative_to(assets); target.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(source,target)


def _source_uri(root, workspace, source_path):
    for item in root.iter('uri'):
        value=(item.text or '').strip()
        if not value: continue
        # The importer packages local mesh/texture resources alongside source.sdf.
        if '://' in value and not value.startswith('file://'): continue
        raw=value.removeprefix('file://')
        candidates=[workspace/raw,source_path.parent/raw]
        for path in candidates:
            path=path.resolve()
            if path.is_file() and path.is_relative_to(workspace.resolve()):
                item.text=path.relative_to(workspace.resolve()).as_posix(); break


def _gazebo(document,workspace,output,report):
    source=document.get('source_sdf')
    if source:
        source_path=_asset(workspace,source); root=ET.parse(source_path).getroot(); model=root.find('model')
        if model is None: raise ValueError('Original SDF must contain a top-level model')
        _source_uri(root,workspace,source_path)
    else:
        root=ET.Element('sdf',version='1.11'); model=ET.SubElement(root,'model')
    root.set('version', max(root.get('version','1.11'), '1.11', key=lambda value: tuple(map(int,value.split('.'))))); model.set('name',document['name'])
    originals={}; original_visuals={}
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type') in set(_TYPES.values()) | {'gps','ray','lidar','gpu_ray','depth'}:
                originals[(link.get('name'),sensor.get('name'))]=copy.deepcopy(sensor); link.remove(sensor)
            else:
                report['warnings'].append('Preserved unsupported source sensor '+str(sensor.get('name'))+' ('+str(sensor.get('type'))+') unchanged in Gazebo.')
        for visual in link.findall('visual'): original_visuals[(link.get('name'),visual.get('name'))]=copy.deepcopy(visual); link.remove(visual)
    links={link.get('name'):link for link in model.findall('link')}
    document_links={link['name'] for link in document['links']}
    for link in list(model.findall('link')):
        if link.get('name') not in document_links: model.remove(link)
    for definition in document['links']:
        link=links.get(definition['name'])
        if link is None: link=ET.SubElement(model,'link',name=definition['name'])
        links[definition['name']]=link
        pose=_set(link,'pose',_nums(definition.get('pose',[0]*6))); pose.attrib.clear()
        _set(link,'inertial/mass',definition.get('mass',1))
        if 'inertial_pose' in definition:
            inertial_pose=_set(link,'inertial/pose',_nums(definition['inertial_pose'])); inertial_pose.attrib.clear()
        for key,value in zip(('ixx','iyy','izz','ixy','ixz','iyz'),definition.get('inertia',[1,1,1,0,0,0])): _set(link,'inertial/inertia/'+key,value)
    for index,visual in enumerate(document['visuals']):
        link=links[visual['link']]
        original=original_visuals.get((visual.get('source_link',visual['link']),visual.get('source_name',visual['name'])))
        node=copy.deepcopy(original) if original is not None else ET.Element('visual')
        node.set('name',visual['name']); link.append(node)
        for old in node.findall('geometry'): node.remove(old)
        pose=_set(node,'pose',_nums(visual.get('pose',[0]*6))); pose.attrib.clear()
        geometry=visual['geometry']; scale=visual.get('scale',[1]*3)
        if geometry['type']=='mesh' or scale != [1,1,1]:
            filename=f'meshes/visual_{index}.obj'; _obj(_meshes(visual,workspace),output/filename)
            _set(node,'geometry/mesh/uri',filename); _set(node,'geometry/mesh/scale',_nums(scale))
        else:
            kind=geometry['type']
            for field in {'box':['size'],'sphere':['radius'],'cylinder':['radius','length']}[kind]:
                value=geometry[field]; _set(node,'geometry/'+kind+'/'+field,_nums(value) if isinstance(value,list) else value)
        if 'color' in visual: _set(node,'material/diffuse',_nums(visual['color'])); _set(node,'material/ambient',_nums(visual['color']))
    systems=set()
    for sensor in document['sensors']:
        if not sensor.get('enabled',True): continue
        original=originals.get((sensor.get('source_parent',sensor['parent']),sensor.get('source_name',sensor['name'])))
        if original is None and sensor.get('xml'): original=ET.fromstring(sensor['xml'])
        node=_sensor_xml(sensor,original)
        if source: _source_uri(node,workspace,source_path)
        links[sensor['parent']].append(node); systems.add(_SYSTEMS[sensor['type']])
    if not source:
        for joint in document.get('joints',[]):
            if joint.get('xml'): model.append(ET.fromstring(joint['xml']))
            else:
                node=ET.SubElement(model,'joint',name=joint['name'],type=joint.get('type','fixed'))
                _set(node,'parent',joint['parent']); _set(node,'child',joint['child']); _set(node,'pose',_nums(joint.get('pose',[0]*6)))
    # Imported plugins are represented in the editable list, including their parent.
    for parent in [model]+list(model.findall('link')):
        for old in list(parent.findall('plugin')): parent.remove(old)
    for plugin in document.get('plugins',[]):
        if plugin.get('target','gazebo') not in ('gazebo','all','*'): continue
        node=ET.fromstring(plugin['xml']) if plugin.get('xml') else ET.Element('plugin')
        if node.tag != 'plugin': raise ValueError('Plugin XML must have a plugin root')
        node.set('name',plugin['name']); node.set('filename',plugin['filename'])
        if source: _source_uri(node,workspace,source_path)
        parent=model if plugin.get('parent','__model__')=='__model__' else links[plugin['parent']]
        parent.append(node)
    ET.indent(root,space='  '); ET.ElementTree(root).write(output/'model.sdf',encoding='utf-8',xml_declaration=True)
    config=ET.Element('model'); _set(config,'name',document['name']); _set(config,'version','1.0'); _set(config,'sdf','model.sdf').set('version',root.get('version'))
    ET.indent(config,space='  '); ET.ElementTree(config).write(output/'model.config',encoding='utf-8',xml_declaration=True)
    world_root=ET.Element('sdf',version='1.11'); world=ET.SubElement(world_root,'world',name='sensor_preview')
    for system in ['Physics','UserCommands','SceneBroadcaster']+sorted(systems):
        stem=re.sub(r'(?<!^)(?=[A-Z])','-',system).lower()
        node=ET.SubElement(world,'plugin',filename='gz-sim-'+stem+'-system',name='gz::sim::systems::'+system)
        if system=='Sensors': _set(node,'render_engine','ogre2')
    _set(world,'gravity','0 0 -9.80665'); _set(world,'magnetic_field','0.000006 0.000023 -0.000042')
    _set(world,'spherical_coordinates/surface_model','EARTH_WGS84'); _set(world,'spherical_coordinates/world_frame_orientation','ENU')
    _set(world,'spherical_coordinates/latitude_deg',0); _set(world,'spherical_coordinates/longitude_deg',0); _set(world,'spherical_coordinates/elevation',0)
    ET.indent(world_root,space='  '); ET.ElementTree(world_root).write(output/'world-systems.sdf',encoding='utf-8',xml_declaration=True)
    report['warnings'] += ['Merge world-systems.sdf systems into your world and set its geographic origin. Model plugins must be installed separately.',
                           'PX4 requires compatible actuator plugins, joints, airframe settings and primary sensor topic names. Export does not establish that integration.']
    if not source: report['warnings'].append('New geometry has no generated collision shapes or flight/vehicle dynamics. Configure them before physical simulation.')


def _xform_lines(matrix,indent,scale=None):
    result=[indent+'double3 xformOp:translate = '+_array([matrix[i][3] for i in range(3)]),
            indent+'quatd xformOp:orient = '+_array(_quat(matrix))]
    order=['xformOp:translate','xformOp:orient']
    if scale is not None: result += [indent+'double3 xformOp:scale = '+_array(scale)]; order += ['xformOp:scale']
    result.append(indent+'uniform token[] xformOpOrder = '+json.dumps(order))
    return result


def _usd_mesh(mesh,name,indent):
    lines=[indent+'def Mesh '+json.dumps(name)+' (prepend apiSchemas = ["MaterialBindingAPI"])','%s{'%indent]
    pad=indent+'    '
    lines += [pad+'point3f[] points = ['+', '.join(_array(p) for p in mesh['vertices'])+']',
              pad+'int[] faceVertexCounts = ['+', '.join(str(len(face)) for face in mesh['faces'])+']',
              pad+'int[] faceVertexIndices = ['+', '.join(str(i) for face in mesh['faces'] for i in face)+']',
              pad+'uniform token subdivisionScheme = "none"',pad+'bool doubleSided = true',
              pad+'color3f[] primvars:displayColor = ['+_array(mesh['color'][:3])+']',
              pad+'float[] primvars:displayOpacity = ['+str(mesh['color'][3])+']']
    if mesh['uv']: lines += [pad+'texCoord2f[] primvars:st = ['+', '.join(_array(p) for p in mesh['uv'])+'] (interpolation = "vertex")']
    lines += [pad+'rel material:binding = <Material>',pad+'def Material "Material"',pad+'{',pad+'    token outputs:surface.connect = <Shader.outputs:surface>',
              pad+'    def Shader "Shader"',pad+'    {',pad+'        uniform token info:id = "UsdPreviewSurface"',
              pad+'        color3f inputs:diffuseColor = '+_array(mesh['color'][:3]),pad+'        float inputs:opacity = '+str(mesh['color'][3]),pad+'        token outputs:surface']
    if mesh['texture'] and mesh['uv']: lines += [pad+'        color3f inputs:diffuseColor.connect = <../Texture.outputs:rgb>']
    lines += [pad+'    }']
    if mesh['texture'] and mesh['uv']:
        lines += [pad+'    def Shader "Texture"',pad+'    {',pad+'        uniform token info:id = "UsdUVTexture"',
                  pad+'        asset inputs:file = @'+mesh['texture']+'@',pad+'        token inputs:sourceColorSpace = "sRGB"',
                  pad+'        float2 inputs:st.connect = <../UV.outputs:result>',pad+'        float3 outputs:rgb',pad+'    }',
                  pad+'    def Shader "UV"',pad+'    {',pad+'        uniform token info:id = "UsdPrimvarReader_float2"',
                  pad+'        token inputs:varname = "st"',pad+'        float2 outputs:result',pad+'    }']
    return lines+[pad+'}',indent+'}']


def _usd(document,workspace,output,report):
    lines=['#usda 1.0','(','    defaultPrim = "Model"','    metersPerUnit = 1','    upAxis = "Z"',')','def Xform "Model"','{']
    # USD cameras look along -Z, +Y up. Canonical cameras look +X, +Z up.
    optical=[[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]]
    for link_index,link in enumerate(document['links']):
        lines += ['    def Xform '+json.dumps('link_'+str(link_index)+'_'+_usd_name(link['name'])),'    {']
        lines += _xform_lines(_matrix(link.get('pose')),'        ')
        lines += ['        custom string xovium:linkName = '+json.dumps(link['name'])]
        for index,visual in enumerate(document['visuals']):
            if visual['link'] != link['name']: continue
            lines += ['        def Xform "visual_'+str(index)+'"','        {']+_xform_lines(_matrix(visual.get('pose')),'            ',visual.get('scale',[1]*3))
            for part,mesh in enumerate(_meshes(visual,workspace)): lines += _usd_mesh(mesh,'part_'+str(part),'            ')
            lines += ['        }']
        for index,sensor in enumerate(document['sensors']):
            if sensor['parent'] != link['name']: continue
            camera=sensor['type'] in ('camera','depth_camera'); p=sensor.get('parameters',{})
            lines += ['        def '+('Camera' if camera else 'Xform')+' "sensor_'+str(index)+'_'+_usd_name(sensor['name'])+'"','        {']
            matrix=_matrix(sensor.get('pose')); matrix=_mul(matrix,optical) if camera else matrix
            lines += _xform_lines(matrix,'            ')
            lines += ['            custom string xovium:sensor = '+json.dumps(json.dumps(sensor,separators=(',',':'))),
                      '            custom bool xovium:enabled = '+str(sensor.get('enabled',True)).lower()]
            if camera:
                width=int(_parameter(p,'width','image_width','image.width',default=640)); height=int(_parameter(p,'height','image_height','image.height',default=480))
                fov=float(_parameter(p,'horizontal_fov','hfov','hfov_rad',default=1.0471975512)); aperture=20.955
                near=float(_parameter(p,'near','near_clip','clip.near',default=.1)); far=float(_parameter(p,'far','far_clip','clip.far',default=100))
                lines += ['            float horizontalAperture = '+str(aperture),'            float verticalAperture = '+str(aperture*height/width),
                          '            float focalLength = '+str(aperture/(2*math.tan(fov/2))),'            float2 clippingRange = '+_array([near,far]),
                          '            custom bool px4:camera:enabled = '+str(sensor.get('enabled',True) and sensor['type']=='camera').lower(),
                          '            custom int px4:camera:width = '+str(width),'            custom int px4:camera:height = '+str(height),
                          '            custom int px4:camera:fps = '+str(max(1,round(sensor.get('rate_hz',30))))]
                if 'port' in p: lines += ['            custom int px4:camera:port = '+str(p['port'])]
                if 'host' in p: lines += ['            custom string px4:camera:host = '+json.dumps(p['host'])]
            lines += ['        }']
        lines += ['    }']
    lines += ['}']; (output/'model.usda').write_text('\n'.join(lines)+'\n')
    report['warnings'] += ['USD preserves link/visual/sensor placement. Joints, collision, mass and vehicle dynamics remain in project.json; no physics adapter is generated.',
                           'RGB Camera prims include Xovium px4:camera settings. Streaming needs the installed px4.isaac.camera extension. Depth cameras need a render product and depth annotator; other sensor types are metadata only.']


def _unreal(document,workspace,output,report):
    links={link['name']:_matrix(link.get('pose')) for link in document['links']}; entries=[]
    for index,visual in enumerate(document['visuals']):
        meshes=_meshes(visual,workspace)
        for mesh in meshes:
            # Import geometry is left handed and centimetres, matching actor poses.
            mesh['vertices']=[[100*p[0],-100*p[1],100*p[2]] for p in mesh['vertices']]
            mesh['faces']=[list(reversed(face)) for face in mesh['faces']]
        filename=f'meshes/visual_{index}.obj'; _obj(meshes,output/filename)
        m=_mul(links[visual['link']],_matrix(visual.get('pose')))
        s=[1,-1,1]; rotation=[[s[i]*m[i][j]*s[j] for j in range(3)] for i in range(3)]
        entries.append({'name':visual['name'],'file':filename,'position':[100*s[i]*m[i][3] for i in range(3)],'quaternion_wxyz':_quat(rotation),'scale':visual.get('scale',[1]*3)})
    cameras=[]
    for sensor in document['sensors']:
        if sensor['type'] not in ('camera','depth_camera') or not sensor.get('enabled',True): continue
        m=_mul(links[sensor['parent']],_matrix(sensor.get('pose'))); s=[1,-1,1]
        rotation=[[s[i]*m[i][j]*s[j] for j in range(3)] for i in range(3)]
        cameras.append({'name':sensor['name'],'position':[100*s[i]*m[i][3] for i in range(3)],'quaternion_wxyz':_quat(rotation),
                        'fov':math.degrees(float(_parameter(sensor.get('parameters',{}),'horizontal_fov','hfov','hfov_rad',default=1.0471975512)))})
    (output/'scene.json').write_text(json.dumps({'name':document['name'],'meshes':entries,'cameras':cameras},indent=2)+'\n')
    (output/'import_unreal.py').write_text('''"""Run in Unreal Editor with Python Editor Script Plugin enabled. Creates actors in the current level."""
from pathlib import Path
import json
import unreal
root = Path(__file__).resolve().parent
scene = json.loads((root / "scene.json").read_text())
folder = "/Game/XoviumModels/" + "".join(c if c.isalnum() or c == "_" else "_" for c in scene["name"])
unreal.EditorAssetLibrary.make_directory(folder)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

def place(kind, entry):
    w, x, y, z = entry["quaternion_wxyz"]
    actor = actors.spawn_actor_from_class(kind, unreal.Vector(*entry["position"]), unreal.Quat(x, y, z, w).rotator())
    actor.set_actor_label(entry["name"])
    return actor

for index, entry in enumerate(scene["meshes"]):
    task = unreal.AssetImportTask()
    task.filename = str(root / entry["file"])
    task.destination_path = folder
    task.destination_name = "visual_" + str(index)
    task.automated = True
    task.save = True
    options = unreal.FbxImportUI()
    options.import_mesh = True
    options.import_as_skeletal = False
    options.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    options.static_mesh_import_data.set_editor_property("convert_scene", False)
    options.static_mesh_import_data.set_editor_property("convert_scene_unit", False)
    options.static_mesh_import_data.set_editor_property("combine_meshes", True)
    task.options = options
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    meshes = [unreal.load_asset(path) for path in task.imported_object_paths]
    mesh = next((asset for asset in meshes if isinstance(asset, unreal.StaticMesh)), None)
    if mesh is None:
        raise RuntimeError("Unreal did not import " + entry["file"])
    actor = place(unreal.StaticMeshActor, entry)
    actor.static_mesh_component.set_static_mesh(mesh)
    actor.set_actor_scale3d(unreal.Vector(*entry["scale"]))
for entry in scene["cameras"]:
    actor = place(unreal.CameraActor, entry)
    actor.get_camera_component().set_field_of_view(entry["fov"])
print("Imported model geometry and camera poses. Configure physics and sensor runtime components separately.")
''')
    report['warnings'] += ['Run import_unreal.py in Unreal Editor with the Python Editor Script Plugin enabled. It creates visual mesh and camera actors in the current level; review and save that level.',
                           'OBJ and scene.json use centimetres and Unreal handedness. Collision, joints, dynamics and all sensor sampling/streaming require runtime components. Camera actors alone do not generate sensor images.']


def _flightgear(document,workspace,output,report):
    links={link['name']:_matrix(link.get('pose')) for link in document['links']}; parts=[]
    for visual in document['visuals']:
        transform=_mul(links[visual['link']],_matrix(visual.get('pose'))); scale=visual.get('scale',[1]*3)
        for mesh in _meshes(visual,workspace):
            mesh['vertices']=[_point(transform,[v[i]*scale[i] for i in range(3)]) for v in mesh['vertices']]; parts.append(mesh)
    lines=['AC3Db']
    for index,mesh in enumerate(parts): lines += [f'MATERIAL "part_{index}" rgb '+_nums(mesh['color'][:3])+' amb 0.2 0.2 0.2 emis 0 0 0 spec 0 0 0 shi 0 trans '+str(1-mesh['color'][3])]
    lines += ['OBJECT world','kids '+str(len(parts))]
    for index,mesh in enumerate(parts):
        lines += ['OBJECT poly','name "part_'+str(index)+'"']
        if mesh['texture']: lines += ['texture '+json.dumps(mesh['texture'])]
        lines += ['numvert '+str(len(mesh['vertices']))]
        # AC3D Y-up; FlightGear model axes X aft, Y right, Z up.
        lines += [_nums([-x,z,y]) for x,y,z in mesh['vertices']]
        lines += ['numsurf '+str(len(mesh['faces']))]
        for face in mesh['faces']:
            lines += ['SURF 0x30','mat '+str(index),'refs '+str(len(face))]
            for vertex in face:
                uv=mesh['uv'][vertex] if mesh['uv'] else [0,0]; lines += [str(vertex)+' '+_nums(uv)]
        lines += ['kids 0']
    (output/'model.ac').write_text('\n'.join(lines)+'\n')
    root=ET.Element('PropertyList'); _set(root,'path','model.ac')
    ET.indent(root,space='  '); ET.ElementTree(root).write(output/'model.xml',encoding='utf-8',xml_declaration=True)
    report['warnings'] += ['model.xml/model.ac are visual model assets, not a complete flyable aircraft. Supply an aircraft -set.xml, FDM and property/telemetry integration separately.',
                           'Sensors and mounting poses are preserved in sensors.json in canonical FLU metres/radians; FlightGear does not execute those sensor definitions.']


def _export_into(document, workspace: Path, target: str, output: Path):
    """Export into a new/empty directory, returning an explicit capability report."""
    workspace=Path(workspace).resolve(); output=Path(output).resolve()
    if target not in ('gazebo','isaac','sih','unreal','flightgear'): raise ValueError('Unknown export target: '+str(target))
    if output == workspace or output.is_relative_to(workspace/'assets'):
        raise ValueError('Export output must not overwrite workspace assets')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('Export directory must be empty')
    for sensor in document.get('sensors',[]):
        if sensor['type'] not in _TYPES: raise ValueError('Unsupported sensor type: '+sensor['type'])
    output.mkdir(parents=True,exist_ok=True)
    report={'format':'xovium-export','version':1,'target':target,'name':document['name'],'files':[],'warnings':[],'sensors':[]}
    for sensor in document.get('sensors',[]):
        camera=sensor['type'] in ('camera','depth_camera')
        status='disabled' if not sensor.get('enabled',True) else ('native_definition' if target=='gazebo' else ('native_camera' if target in ('isaac','sih','unreal') and camera else 'metadata_only'))
        report['sensors'].append({'id':sensor['id'],'name':sensor['name'],'type':sensor['type'],'status':status,'runtime_integration_required':status!='disabled'})
    _copy_assets(workspace,output)
    (output/'project.json').write_text(json.dumps(document,indent=2)+'\n')
    (output/'sensors.json').write_text(json.dumps({'frame':'FLU','units':'m','angles':'radians','sensors':document.get('sensors',[]),'plugins':document.get('plugins',[])},indent=2)+'\n')
    {'gazebo':_gazebo,'isaac':_usd,'sih':_usd,'unreal':_unreal,'flightgear':_flightgear}[target](document,workspace,output,report)
    report['files']=sorted([p.relative_to(output).as_posix() for p in output.rglob('*') if p.is_file()]+['report.json'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def export_project(document, workspace: Path, target: str, output: Path):
    """Publish a complete export atomically; leave no partial files on failure."""
    workspace=Path(workspace).resolve(); output=Path(output).absolute()
    if output.is_symlink(): raise ValueError('Export directory must not be a symbolic link')
    output=output.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('Export directory must be empty')
    if output == workspace or output.is_relative_to(workspace/'assets'):
        raise ValueError('Export output must not overwrite workspace assets')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.xovium-export-',dir=output.parent) as temporary:
        stage=Path(temporary)/'export'
        result=_export_into(document,workspace,target,stage)
        if output.exists():
            if not output.is_dir() or any(output.iterdir()): raise ValueError('Export directory changed during export')
            output.rmdir()
        stage.replace(output)
    return result
