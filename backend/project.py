"""Portable model documents, SDF/mesh import and strict validation (stdlib only)."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import shlex
import stat
import subprocess
import tempfile
import uuid
import xml.etree.ElementTree as ET
import zipfile

FORMAT = 'xovium-model'
VERSION = 1
MAX_FILES = 10000
MAX_BYTES = 2 * 1024**3
MAX_FILE_BYTES = 512 * 1024**2
CATALOG = json.loads(Path(__file__).with_name('sensors.json').read_text())
SENSOR_TYPES = {item['id']: item for item in CATALOG['types']}

class ProjectError(ValueError):
    pass


def _json(path):
    if not Path(path).is_file():raise ProjectError('Expected a regular JSON file: '+str(path))
    if Path(path).stat().st_size > MAX_FILE_BYTES:
        raise ProjectError('JSON asset is too large: ' + str(path))
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ProjectError('Non-finite JSON number: '+value)))


def _text(value, label, empty=False):
    if not isinstance(value, str) or (not empty and not value.strip()) or any(ord(c)<32 for c in value) or len(value)>4096:
        raise ProjectError(label+' must be a valid '+('string' if empty else 'nonempty string'))
    return value


def _number(value, label, minimum=None, maximum=None, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
        raise ProjectError(label+' must be a finite number')
    if integer and not isinstance(value,int): raise ProjectError(label+' must be an integer')
    if minimum is not None and value < minimum: raise ProjectError(label+' is below '+str(minimum))
    if maximum is not None and value > maximum: raise ProjectError(label+' is above '+str(maximum))
    return value


def _vector(value, count, label, minimum=None, maximum=None):
    if not isinstance(value,list) or len(value)!=count: raise ProjectError(f'{label} must contain {count} numbers')
    for n in value: _number(n,label,minimum,maximum)
    return value


def asset_path(value, workspace=None, must_exist=True):
    _text(value,'Asset path')
    path=PurePosixPath(value)
    if '\\' in value or ':' in value or path.is_absolute() or any(p in ('','..','.') for p in value.split('/')) or path.parts[0]!='assets':
        raise ProjectError('Asset path must stay inside assets/: '+value)
    if workspace is None: return path
    root=Path(workspace).resolve()
    result=root.joinpath(*path.parts)
    current=root
    for part in path.parts:
        current=current/part
        if current.is_symlink(): raise ProjectError('Asset symlinks are not allowed: '+value)
    if not result.resolve().is_relative_to(root): raise ProjectError('Asset escapes workspace: '+value)
    if must_exist and not result.is_file(): raise ProjectError('Missing project asset: '+value)
    return result


def _xml(text):
    if not isinstance(text,str) or len(text)>16*1024**2: raise ProjectError('XML must be a string under 16 MiB')
    if '<!DOCTYPE' in text.upper() or '<!ENTITY' in text.upper(): raise ProjectError('XML entities and DTDs are not supported')
    try: return ET.fromstring(text)
    except ET.ParseError as error: raise ProjectError('Invalid XML: '+str(error)) from error


def validate_mesh(meshes, workspace=None):
    if not isinstance(meshes,list) or not meshes: raise ProjectError('Mesh JSON must contain a nonempty mesh list')
    for m in meshes:
        if not isinstance(m,dict): raise ProjectError('Invalid mesh record')
        vertices=m.get('vertices'); faces=m.get('faces')
        if not isinstance(vertices,list) or not vertices or not isinstance(faces,list) or not faces: raise ProjectError('Mesh requires vertices and triangle faces')
        for p in vertices: _vector(p,3,'Mesh vertex')
        for face in faces:
            if not isinstance(face,list) or len(face)!=3: raise ProjectError('Mesh faces must be triangles')
            for i in face: _number(i,'Triangle index',0,len(vertices)-1,integer=True)
        uv=m.get('uv',[])
        if not isinstance(uv,list) or len(uv) not in (0,len(vertices)): raise ProjectError('Mesh UVs must match vertices')
        for p in uv: _vector(p,2,'Texture coordinate')
        _vector(m.get('color',[.7,.7,.7,1]),4,'Mesh color',0,1)
        if m.get('texture'): asset_path(m['texture'],workspace)
        if not isinstance(m.get('dependencies',[]),list):raise ProjectError('Mesh dependencies must be a list')
        for dependency in m.get('dependencies',[]):asset_path(dependency,workspace)
    return meshes


def validate_project(document, workspace=None):
    if not isinstance(document,dict) or document.get('format')!=FORMAT or (type(document.get('version')) is not int or document.get('version')!=VERSION):
        raise ProjectError('Unsupported model project format or version')
    _text(document.get('name'),'Project name')
    if document.get('vehicle') not in ('fixed-wing','rover','custom'): raise ProjectError('Vehicle must be fixed-wing, rover or custom')
    if document.get('frame')!='FLU' or document.get('units')!='m': raise ProjectError('Projects use FLU axes and metres')
    for key in ('links','visuals','sensors','plugins'):
        if not isinstance(document.get(key),list): raise ProjectError(key+' must be a list')
    if not document['links']: raise ProjectError('Project needs at least one link')
    link_names=set(); ids=set(); names={}
    def unique(item,kind,parent=''):
        if not isinstance(item,dict): raise ProjectError('Invalid '+kind+' record')
        name=_text(item.get('name'),kind+' name')
        if any(c in name for c in '/\\<>'): raise ProjectError('Invalid '+kind+' name: '+name)
        marker=(kind,parent,name)
        if marker in names and kind!='plugin': raise ProjectError('Duplicate '+kind+' name: '+name)
        names[marker]=True
        if kind!='link' and kind!='joint':
            identifier=_text(item.get('id'),kind+' ID')
            if identifier in ids: raise ProjectError('Duplicate object ID: '+identifier)
            ids.add(identifier)
    for link in document['links']:
        unique(link,'link');link_names.add(link['name'])
        _vector(link.get('pose'),6,'Link pose')
        _number(link.get('mass'),'Link mass',1e-12,1e12)
        a,b,c,d,e,f=_vector(link.get('inertia'),6,'Link inertia')
        if min(a,b,c)<=0 or a*b-d*d<=0 or a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d<=0:
            raise ProjectError('Link inertia must be a positive-definite symmetric tensor')
        if 'inertial_pose' in link: _vector(link['inertial_pose'],6,'Inertial pose')
    for visual in document['visuals']:
        unique(visual,'visual',visual.get('link',''))
        if visual.get('link') not in link_names: raise ProjectError('Visual has an unknown parent link')
        _vector(visual.get('pose'),6,'Visual pose');_vector(visual.get('scale'),3,'Visual scale',1e-9,1e9)
        _vector(visual.get('color',[.7,.7,.7,1]),4,'Visual color',0,1)
        geometry=visual.get('geometry')
        if not isinstance(geometry,dict): raise ProjectError('Visual requires geometry')
        kind=geometry.get('type')
        if kind=='mesh':
            path=asset_path(geometry.get('path'),workspace)
            if workspace is not None: validate_mesh(_json(path),workspace)
        elif kind=='box': _vector(geometry.get('size'),3,'Box size',1e-9,1e9)
        elif kind in ('sphere','cylinder'):
            _number(geometry.get('radius'),'Radius',1e-9,1e9)
            if kind=='cylinder':_number(geometry.get('length'),'Cylinder length',1e-9,1e9)
        else:raise ProjectError('Unsupported geometry type: '+str(kind))
    for sensor in document['sensors']:
        unique(sensor,'sensor',sensor.get('parent',''))
        if sensor.get('parent') not in link_names: raise ProjectError('Sensor has an unknown parent link')
        if sensor.get('type') not in SENSOR_TYPES: raise ProjectError('Unsupported sensor type: '+str(sensor.get('type')))
        _vector(sensor.get('pose'),6,'Sensor pose')
        if not isinstance(sensor.get('enabled'),bool):raise ProjectError('Sensor enabled must be boolean')
        _number(sensor.get('rate_hz'),'Sensor update rate',.01,10000)
        _text(sensor.get('topic',''),'Sensor topic',empty=True)
        parameters=sensor.get('parameters')
        if not isinstance(parameters,dict):raise ProjectError('Sensor parameters must be an object')
        definitions={p['key']:p for p in SENSOR_TYPES[sensor['type']]['parameters']}
        for key,value in parameters.items():
            if key not in definitions:raise ProjectError('Unknown '+sensor['type']+' parameter: '+key)
            spec=definitions[key]
            if spec['type'] in ('int','float'):_number(value,key,spec.get('min'),spec.get('max'),spec['type']=='int')
            elif spec['type']=='string':_text(value,key,empty=True)
            elif not isinstance(value,bool):raise ProjectError(key+' must be boolean')
        params={**SENSOR_TYPES[sensor['type']]['default_parameters'],**parameters}
        for low,high in (('near','far'),('min_range','max_range')):
            if low in params and params[low]>=params[high]:raise ProjectError(high+' must exceed '+low)
        for axis in ('horizontal','vertical'):
            low,high=axis+'_min_angle',axis+'_max_angle'
            if low in params and params[low]>params[high]:raise ProjectError(high+' must be at least '+low)
        if 'xml' in sensor:_xml(sensor['xml'])
    for plugin in document['plugins']:
        unique(plugin,'plugin',plugin.get('parent','__model__'))
        if plugin.get('parent','__model__') not in link_names|{'__model__'}:raise ProjectError('Plugin has an unknown parent link')
        for key in ('target','filename'):_text(plugin.get(key),'Plugin '+key,empty=key=='filename')
        if _xml(plugin['xml']).tag!='plugin':raise ProjectError('Plugin XML must contain a plugin element')
    joints=document.get('joints',[])
    if not isinstance(joints,list):raise ProjectError('Joints must be a list')
    for joint in joints:
        unique(joint,'joint')
        if joint['name'] in link_names:raise ProjectError('Joint and link frame names must be distinct')
        if joint.get('parent') not in link_names|{'world'} or joint.get('child') not in link_names:raise ProjectError('Joint has an unknown parent or child')
        _text(joint.get('type'),'Joint type');_vector(joint.get('pose'),6,'Joint pose')
        if 'xml' in joint and _xml(joint['xml']).tag!='joint':raise ProjectError('Joint XML must contain a joint element')
    if 'source_sdf' in document:
        p=asset_path(document['source_sdf'],workspace)
        if workspace is not None:
            source_tree=_xml(p.read_text())
            for element in source_tree.iter():
                if element.tag not in ('uri','albedo_map','normal_map','roughness_map','metalness_map','emissive_map','environment_map') or not (element.text or '').strip():continue
                value=element.text.strip()
                if value.startswith(('__default__','name://')):continue
                if '://' in value or Path(value).is_absolute() or '\\' in value:raise ProjectError('Source SDF resource must be a bundled relative path: '+value)
                target=(p.parent/value).resolve()
                if not target.is_relative_to(Path(workspace).resolve()/'assets') or not target.is_file():raise ProjectError('Missing or nonportable source SDF resource: '+value)
                asset_path(target.relative_to(Path(workspace).resolve()).as_posix(),workspace)
    if 'warnings' in document and (not isinstance(document['warnings'],list) or not all(isinstance(w,str) for w in document['warnings'])):raise ProjectError('Warnings must be a string list')
    return document


def new_project(name='Untitled model', vehicle='fixed-wing'):
    size=[.8,1.4,.15] if vehicle=='fixed-wing' else [.4,.25,.25]
    mass=1.5 if vehicle=='fixed-wing' else 20.0 if vehicle=='rover' else 1.0
    x,y,z=size
    document={'format':FORMAT,'version':VERSION,'name':name,'vehicle':vehicle,'frame':'FLU','units':'m','links':[{'name':'base_link','pose':[0.0]*6,'mass':mass,'inertia':[mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12,0,0,0]}],'visuals':[{'id':str(uuid.uuid4()),'name':'body','link':'base_link','pose':[0.0]*6,'scale':[1,1,1],'geometry':{'type':'box','size':size},'color':[.2,.55,.85,1]}],'sensors':[],'plugins':[],'joints':[],'warnings':[]}
    return validate_project(document)


def load_workspace(workspace):
    root=Path(workspace)
    if root.is_symlink() or (root/'project.json').is_symlink():raise ProjectError('Workspace and project file must not be symlinks')
    return validate_project(_json(root/'project.json'),root)


def write_workspace(document,workspace):
    root=Path(workspace)
    if root.is_symlink():raise ProjectError('Workspace must not be a symlink')
    root.mkdir(parents=True,exist_ok=True)
    if (root/'assets').is_symlink():raise ProjectError('Assets directory must not be a symlink')
    (root/'assets').mkdir(exist_ok=True)
    validate_project(document,root)
    _atomic_json(root/'project.json',document)
    return root


def _atomic_json(path,data):
    path=Path(path)
    if path.is_symlink():raise ProjectError('Refusing to replace a symlink: '+str(path))
    with tempfile.NamedTemporaryFile('w',dir=path.parent,prefix='.'+path.name+'.',delete=False) as file:
        temporary=Path(file.name)
        try:json.dump(data,file,indent=2,allow_nan=False);file.write('\n')
        except BaseException:temporary.unlink(missing_ok=True);raise
    try:temporary.replace(path)
    finally:temporary.unlink(missing_ok=True)


def _safe_files(root):
    if root.is_symlink():raise ProjectError('Asset root must not be a symlink')
    files=[];total=0
    for path in root.rglob('*'):
        if path.is_symlink():raise ProjectError('Symlinks are not allowed in a model bundle')
        if path.is_dir():continue
        if not path.is_file():raise ProjectError('Only regular files can be bundled')
        size=path.stat().st_size
        if size>MAX_FILE_BYTES:raise ProjectError('Asset exceeds the 512 MiB size limit')
        total+=size;files.append(path)
        if total>MAX_BYTES or len(files)>MAX_FILES:raise ProjectError('Model bundle exceeds size/file limits')
    return files


def save_project(workspace,output):
    root=Path(workspace).resolve();document=load_workspace(root);output=Path(output).absolute()
    if output.suffix.lower()!='.xovmodel':raise ProjectError('Save filename must end in .xovmodel')
    if output.is_symlink() or output.resolve().is_relative_to(root/'assets'):raise ProjectError('Save outside the project assets directory')
    files=_safe_files(root/'assets')
    output.parent.mkdir(parents=True,exist_ok=True)
    handle=tempfile.NamedTemporaryFile(dir=output.parent,prefix='.'+output.name+'.',delete=False);temp=Path(handle.name);handle.close()
    try:
        with zipfile.ZipFile(temp,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
            archive.writestr('project.json',json.dumps(document,indent=2,allow_nan=False)+'\n')
            for path in sorted(files):archive.write(path,path.relative_to(root).as_posix())
        temp.replace(output)
    finally:temp.unlink(missing_ok=True)
    return output


def _unpack(source,stage):
    with zipfile.ZipFile(source) as archive:
        total=0;seen=set()
        if len(archive.infolist())>MAX_FILES:raise ProjectError('Project archive has too many entries')
        for info in archive.infolist():
            name=info.filename;path=PurePosixPath(name);parts=name.rstrip('/').split('/')
            mode=info.external_attr>>16
            if name.startswith('/') or '\\' in name or ':' in name or any(p in ('','..','.') for p in parts) or (parts[0]!='assets' and name!='project.json'):
                raise ProjectError('Unsafe project archive path: '+name)
            if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0,stat.S_IFREG,stat.S_IFDIR)):raise ProjectError('Archive links and special files are not allowed')
            if name.rstrip('/') in seen:raise ProjectError('Duplicate archive path: '+name)
            seen.add(name.rstrip('/'));total+=info.file_size
            if info.file_size>MAX_FILE_BYTES or total>MAX_BYTES:raise ProjectError('Project archive exceeds size limits')
            target=stage.joinpath(*parts)
            if info.is_dir():target.mkdir(parents=True,exist_ok=True);continue
            target.parent.mkdir(parents=True,exist_ok=True)
            with archive.open(info) as src,target.open('wb') as dst:shutil.copyfileobj(src,dst)
    return load_workspace(stage)


def _matrix(pose):
    x,y,z,r,p,h=pose;cr,sr,cp,sp,ch,sh=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(h),math.sin(h)
    return [[ch*cp,ch*sp*sr-sh*cr,ch*sp*cr+sh*sr,x],[sh*cp,sh*sp*sr+ch*cr,sh*sp*cr-ch*sr,y],[-sp,cp*sr,cp*cr,z],[0,0,0,1]]

def _multiply(a,b):return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def _inverse(a):
    out=[[a[j][i] for j in range(3)]+[0] for i in range(3)]+[[0,0,0,1]]
    for i in range(3):out[i][3]=-sum(out[i][j]*a[j][3] for j in range(3))
    return out

def _pose(matrix):
    pitch=math.asin(max(-1,min(1,-matrix[2][0])))
    if abs(math.cos(pitch))>1e-8:r,y=math.atan2(matrix[2][1],matrix[2][2]),math.atan2(matrix[1][0],matrix[0][0])
    else:r,y=0,math.atan2(-matrix[0][1],matrix[1][1])
    return [matrix[0][3],matrix[1][3],matrix[2][3],r,pitch,y]

def _numbers(text,count,default=None):
    try:values=[float(n) for n in (text or default or '').split()]
    except ValueError as error:raise ProjectError('Invalid numeric SDF value: '+str(text)) from error
    return _vector(values,count,'SDF numeric value')

def _color(text):
    values=[float(n) for n in (text or '0.7 0.7 0.7 1').split()]
    if len(values)==3:values.append(1.0)
    return _vector(values,4,'SDF visual color',0,1)

def _sdf_pose(element,parent):
    p=element.find('pose')
    if p is None:return parent,_matrix([0]*6)
    if p.get('rotation_format','euler_rpy')!='euler_rpy':raise ProjectError('Quaternion SDF poses are not supported')
    values=_numbers(p.text,6,'0 0 0 0 0 0')
    if p.get('degrees','false').lower()=='true':values[3:]=[math.radians(v) for v in values[3:]]
    return p.get('relative_to') or p.findtext('relative_to') or parent,_matrix(values)


class _Importer:
    def __init__(self,source,stage,helper):
        self.source=Path(source).resolve();self.root=self.source.parent;self.stage=stage
        self.helper=Path(helper).resolve() if helper else None;self.mesh_cache={};self.copied={};self.texture_dirs={self.root}
        self.warnings=[]
    def local(self,uri):
        from urllib.parse import unquote
        uri=unquote((uri or '').strip()).replace('\\','/')
        if uri.startswith('model://'):
            model,_,relative=uri[8:].partition('/')
            if model not in (self.root.name,getattr(self,'model_name','')):raise ProjectError('External model resource is not bundled: '+uri)
            path=self.root/relative
        elif uri.startswith('file://'):path=Path(uri[7:]);path=path if path.is_absolute() else self.root/path
        elif '://' in uri:raise ProjectError('Remote resource is not supported; download it into the model folder: '+uri)
        else:path=self.root/uri
        return self.allowed(path)
    def allowed(self,path,extra=None):
        path=Path(path)
        absolute=path.absolute()
        for parent in [absolute,*absolute.parents]:
            if parent==self.root.parent:break
            if parent.is_symlink():raise ProjectError('Source symlinks are not supported: '+str(path))
        path=path.resolve()
        if not path.is_relative_to(self.root) and not (extra and path.is_relative_to(extra)):
            raise ProjectError('Resource escapes the source model folder: '+str(path))
        if not path.is_file():raise ProjectError('Missing source asset: '+str(path))
        if path.stat().st_size>MAX_FILE_BYTES:raise ProjectError('Source asset is too large')
        return path
    def copy_source(self,path):
        path=self.allowed(path)
        relative=Path('assets/source')/path.relative_to(self.root)
        destination=self.stage/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        if not destination.exists():shutil.copy2(path,destination)
        return relative.as_posix()
    def sidecars(self,path):
        if path.suffix.lower()=='.obj':
            for line in path.read_text(errors='replace').splitlines():
                if not line.lstrip().startswith('mtllib '):continue
                for item in shlex.split(line.strip())[1:]:
                    material=self.allowed(path.parent/item);self.copy_source(material);self.texture_dirs.add(material.parent)
                    for entry in material.read_text(errors='replace').splitlines():
                        fields=shlex.split(entry,comments=True)
                        if fields and (fields[0].lower().startswith('map_') or fields[0].lower() in ('bump','disp','decal','norm')):
                            texture=self.allowed(material.parent/fields[-1]);self.copy_source(texture)
        if path.suffix.lower()=='.gltf':
            from urllib.parse import unquote
            data=_json(path)
            for record in data.get('buffers',[])+data.get('images',[]):
                uri=record.get('uri','')
                if uri and not uri.startswith('data:'):
                    source=self.allowed(path.parent/unquote(uri));self.copy_source(source)
                    record['uri']=os.path.relpath(source,path.parent)
            _atomic_json(self.stage/self.copy_source(path),data)
        if path.suffix.lower()=='.dae':
            from urllib.parse import unquote
            tree=_xml(path.read_text())
            for element in tree.iter():
                if element.tag.rsplit('}',1)[-1]!='image':continue
                for ref in element.iter():
                    if ref.tag.rsplit('}',1)[-1]!='init_from' or not (ref.text or '').strip():continue
                    raw=unquote(ref.text.strip()).removeprefix('file://')
                    source=self.allowed(path.parent/raw);self.copy_source(source)
                    ref.text=os.path.relpath(source,path.parent)
            if tree.tag.startswith('{'):ET.register_namespace('',tree.tag[1:].split('}',1)[0])
            ET.ElementTree(tree).write(self.stage/self.copy_source(path),encoding='unicode',xml_declaration=True)
    def texture(self,path,extra=None):
        path=self.allowed(path,extra)
        digest=hashlib.sha256(path.read_bytes()).hexdigest()[:20]
        relative=Path('assets/textures')/(digest+path.suffix.lower())
        destination=self.stage/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        if not destination.exists():shutil.copy2(path,destination)
        if path.is_relative_to(self.root):self.copy_source(path)
        return relative.as_posix()
    def mesh(self,path):
        path=self.allowed(path)
        if path in self.mesh_cache:return self.mesh_cache[path]
        if not self.helper or not self.helper.is_file():raise ProjectError('Mesh import requires --mesh-helper or XOVIUM_MESH_IMPORT')
        self.copy_source(path);self.sidecars(path)
        with tempfile.TemporaryDirectory(prefix='mesh-',dir=self.stage) as temp:
            temp=Path(temp);output=temp/'mesh.json'
            result=subprocess.run([str(self.helper),str(path),str(output)],capture_output=True,text=True,timeout=120)
            if result.returncode:raise ProjectError('Mesh import failed: '+(result.stderr.strip() or result.stdout.strip() or str(result.returncode))[-2000:])
            meshes=_json(output)
            if not isinstance(meshes,list):raise ProjectError('Mesh helper returned an invalid document')
            for mesh in meshes:
                texture=mesh.get('texture','')
                if texture:
                    if texture.startswith('*'):raise ProjectError('Embedded texture was not extracted by the mesh helper')
                    texture_path=Path(texture.replace('\\','/'))
                    if not texture_path.is_absolute():
                        candidates=[path.parent/texture_path]+[directory/texture_path for directory in self.texture_dirs]
                        texture_path=next((candidate for candidate in candidates if candidate.is_file()),candidates[0])
                    mesh['texture']=self.texture(texture_path,temp)
                dependencies=[]
                for dependency in mesh.get('dependencies',[]):
                    dep=Path(dependency.replace('\\','/'))
                    if not dep.is_absolute():
                        candidates=[path.parent/dep]+[directory/dep for directory in self.texture_dirs]
                        dep=next((candidate for candidate in candidates if candidate.is_file()),candidates[0])
                    dependencies.append(self.texture(dep,temp))
                if dependencies:mesh['dependencies']=dependencies
            validate_mesh(meshes,self.stage)
            relative='assets/meshes/'+hashlib.sha256(str(path.relative_to(self.root)).encode()).hexdigest()[:20]+'.json'
            (self.stage/relative).parent.mkdir(parents=True,exist_ok=True)
            _atomic_json(self.stage/relative,meshes)
        self.mesh_cache[path]=relative
        return relative
    def retained_resources(self,tree):
        # Preserve collision geometry, plugin URIs and PBR texture resources too.
        tags={'uri','albedo_map','normal_map','roughness_map','metalness_map','emissive_map','environment_map'}
        for element in tree.iter():
            if element.tag not in tags or not (element.text or '').strip():continue
            value=element.text.strip()
            if value.startswith(('__default__','name://')):continue
            source=self.local(value);relative=self.copy_source(source)
            element.text=relative.removeprefix('assets/')
            if source.suffix.lower() in ('.obj','.dae','.gltf','.glb','.fbx','.stl'):
                self.mesh(source)
        for path in self.root.iterdir():
            if path.is_file() and path.name.upper().startswith(('LICENSE','COPYING','NOTICE')):self.copy_source(path)


def _sensor(element,parent,pose):
    aliases={'air_speed':'airspeed','navsat':'gps','air_pressure':'barometer','gpu_lidar':'lidar','gpu_ray':'lidar','ray':'lidar','depth':'depth_camera'}
    kind=aliases.get(element.get('type'),element.get('type'))
    if kind not in SENSOR_TYPES:return None
    spec=SENSOR_TYPES[kind];params=copy.deepcopy(spec['default_parameters'])
    reads={
      'camera':{'width':'camera/image/width','height':'camera/image/height','horizontal_fov':'camera/horizontal_fov','near':'camera/clip/near','far':'camera/clip/far'},
      'lidar':{'horizontal_samples':'lidar/scan/horizontal/samples','horizontal_min_angle':'lidar/scan/horizontal/min_angle','horizontal_max_angle':'lidar/scan/horizontal/max_angle','vertical_samples':'lidar/scan/vertical/samples','vertical_min_angle':'lidar/scan/vertical/min_angle','vertical_max_angle':'lidar/scan/vertical/max_angle','min_range':'lidar/range/min','max_range':'lidar/range/max'},
      'barometer':{'pressure_noise':'air_pressure/pressure/noise/stddev'},'airspeed':{'noise':'air_speed/pressure/noise/stddev'},'imu':{'accel_noise':'imu/linear_acceleration/x/noise/stddev','gyro_noise':'imu/angular_velocity/x/noise/stddev'},'magnetometer':{'noise':'magnetometer/x/noise/stddev'},'gps':{'position_noise':'navsat/position_sensing/horizontal/noise/stddev','velocity_noise':'navsat/velocity_sensing/horizontal/noise/stddev'},'contact':{'collision':'contact/collision'}}
    for key,path in reads.get('camera' if kind=='depth_camera' else kind,{}).items():
        value=element.findtext(path)
        if value is None and path.startswith('lidar/'):value=element.findtext(path.replace('lidar/','ray/',1))
        if value is not None:params[key]=str(value) if isinstance(params[key],str) else int(value) if isinstance(params[key],int) else float(value)
    rate=float(element.findtext('update_rate',str(spec['default_rate_hz'])))
    if rate<=0:rate=spec['default_rate_hz']
    name=element.get('name',kind)
    return {'id':str(uuid.uuid4()),'name':name,'source_name':name,'source_parent':parent,'type':kind,'parent':parent,'pose':pose,'enabled':True,'rate_hz':rate,'source_rate_hz':rate,'topic':element.findtext('topic',''),'parameters':params,'source_parameters':copy.deepcopy(params),'xml':ET.tostring(element,encoding='unicode')}


def _import_sdf(source,stage,helper):
    importer=_Importer(source,stage,helper);root=_xml(Path(source).read_text())
    if root.tag=='model':
        model_root=root;root=ET.Element('sdf',version='1.11');root.append(model_root)
    models=root.findall('model')
    if len(models)!=1:raise ProjectError('Import requires exactly one top-level SDF model')
    model=models[0];importer.model_name=model.get('name','Imported model')
    if model.find('include') is not None or model.find('model') is not None:raise ProjectError('Nested SDF models/includes are not supported; import a self-contained model')
    document=new_project(importer.model_name,'rover' if 'rover' in importer.model_name.lower() else 'fixed-wing' if any(n in importer.model_name.lower() for n in ('cessna','plane','aircraft')) else 'custom')
    document.update(links=[],visuals=[],sensors=[],plugins=[],joints=[])
    preserved=[]
    nodes={}
    def add(name,value):
        if not name or name in nodes or name=='__model__':raise ProjectError('Duplicate/invalid SDF frame name: '+str(name))
        nodes[name]=value
    for link in model.findall('link'):add(link.get('name'),_sdf_pose(link,'__model__'))
    for joint in model.findall('joint'):add(joint.get('name'),_sdf_pose(joint,joint.findtext('child','__model__')))
    for frame in model.findall('frame'):add(frame.get('name'),_sdf_pose(frame,frame.get('attached_to','__model__')))
    resolved={'__model__':_matrix([0]*6)};visiting=set()
    def resolve(name):
        if name==importer.model_name and name not in nodes:name='__model__'
        if name in resolved:return resolved[name]
        if name not in nodes:raise ProjectError('Unknown SDF pose frame: '+name)
        if name in visiting:raise ProjectError('Cyclic SDF frame reference: '+name)
        visiting.add(name);parent,local=nodes[name];resolved[name]=_multiply(resolve(parent),local);visiting.remove(name)
        return resolved[name]
    def local_pose(element,parent):
        frame,local=_sdf_pose(element,parent)
        return _pose(_multiply(_inverse(resolve(parent)),_multiply(resolve(frame),local)))
    for name in nodes:resolve(name)
    for link in model.findall('link'):
        name=link.get('name');inertial=link.find('inertial')
        mass=float(link.findtext('inertial/mass','1'))
        inertia=[float(link.findtext('inertial/inertia/'+key,'1' if key in ('ixx','iyy','izz') else '0')) for key in ('ixx','iyy','izz','ixy','ixz','iyz')]
        record={'name':name,'pose':_pose(resolve(name)),'mass':mass,'inertia':inertia}
        if inertial is not None:record['inertial_pose']=local_pose(inertial,name)
        else:document['warnings'].append('Link '+name+' has no inertial block; editor defaults to 1 kg and unit inertia.')
        document['links'].append(record)
        for visual in link.findall('visual'):
            geom=visual.find('geometry')
            if geom is None:raise ProjectError('Visual has no geometry: '+str(visual.get('name')))
            scale=[1,1,1];geometry=None
            if geom.find('mesh') is not None:
                mesh=geom.find('mesh')
                if mesh.find('submesh') is not None:raise ProjectError('SDF mesh sub-selection is not supported')
                geometry={'type':'mesh','path':importer.mesh(importer.local(mesh.findtext('uri')))}
                scale=_numbers(mesh.findtext('scale'),3,'1 1 1')
            elif geom.find('box') is not None:geometry={'type':'box','size':_numbers(geom.findtext('box/size'),3)}
            elif geom.find('sphere') is not None:geometry={'type':'sphere','radius':float(geom.findtext('sphere/radius'))}
            elif geom.find('cylinder') is not None:geometry={'type':'cylinder','radius':float(geom.findtext('cylinder/radius')),'length':float(geom.findtext('cylinder/length'))}
            else:raise ProjectError('Unsupported SDF visual geometry: '+ET.tostring(geom,encoding='unicode')[:150])
            visual_name=visual.get('name','visual_'+str(len(document['visuals'])))
            document['visuals'].append({'id':str(uuid.uuid4()),'name':visual_name,'source_name':visual_name,'source_link':name,'link':name,'pose':local_pose(visual,name),'scale':scale,'geometry':geometry,'color':_color(visual.findtext('material/diffuse','1 1 1 1' if geometry['type']=='mesh' else '0.7 0.7 0.7 1'))})
        for element in link.findall('sensor'):
            sensor=_sensor(element,name,local_pose(element,name))
            if sensor is None:document['warnings'].append('Unsupported sensor '+str(element.get('type'))+' retained in source SDF only.')
            else:document['sensors'].append(sensor);preserved.append((sensor,element))
    for parent,owner in [('__model__',model)]+[(link.get('name'),link) for link in model.findall('link')]:
        for element in owner.findall('plugin'):
            document['plugins'].append({'id':str(uuid.uuid4()),'name':element.get('name','plugin_'+str(len(document['plugins']))),'target':'gazebo','parent':parent,'filename':element.get('filename',''),'xml':ET.tostring(element,encoding='unicode')});preserved.append((document['plugins'][-1],element))
    for joint in model.findall('joint'):
        document['joints'].append({'name':joint.get('name'),'type':joint.get('type','fixed'),'parent':joint.findtext('parent'),'child':joint.findtext('child'),'pose':_pose(resolve(joint.get('name'))),'xml':ET.tostring(joint,encoding='unicode')});preserved.append((document['joints'][-1],joint))
    importer.retained_resources(root)
    for record,element in preserved:record['xml']=ET.tostring(element,encoding='unicode')
    (stage/'assets').mkdir(exist_ok=True)
    ET.ElementTree(root).write(stage/'assets/source.sdf',encoding='unicode',xml_declaration=True)
    document['source_sdf']='assets/source.sdf'
    return validate_project(document,stage)


def import_project(source,workspace,mesh_helper=None):
    source=Path(source).absolute();workspace=Path(workspace).absolute()
    if source.is_symlink() or not source.is_file():raise ProjectError('Import source must be a regular file')
    if workspace.is_symlink():raise ProjectError('Workspace must not be a symlink')
    if workspace.exists() and any(p.name not in ('project.json','assets') for p in workspace.iterdir()):raise ProjectError('Import workspace contains unrelated files')
    if workspace.exists() and any(workspace.iterdir()) and not (workspace/'project.json').is_file():raise ProjectError('Import workspace must be empty or an existing model project')
    workspace.parent.mkdir(parents=True,exist_ok=True)
    helper=mesh_helper or os.environ.get('XOVIUM_MESH_IMPORT')
    with tempfile.TemporaryDirectory(prefix='.xovmodel-import-',dir=workspace.parent) as temporary:
        stage=Path(temporary)/'workspace';stage.mkdir();(stage/'assets').mkdir()
        if source.suffix.lower()=='.xovmodel':document=_unpack(source,stage)
        elif source.suffix.lower()=='.sdf':document=_import_sdf(source,stage,helper)
        elif source.suffix.lower() in ('.obj','.dae','.stl','.fbx','.gltf','.glb'):
            importer=_Importer(source,stage,helper);document=new_project(source.stem,'custom')
            document['visuals'][0].update(name=source.stem,geometry={'type':'mesh','path':importer.mesh(source)},color=[1,1,1,1])
            document['warnings'].append('Mesh import supplies visual geometry; configure mass, inertia, collision and sensors before simulation.')
        else:raise ProjectError('Supported imports: .xovmodel, .sdf, .obj, .dae, .stl, .fbx, .gltf, .glb')
        write_workspace(document,stage);_safe_files(stage)
        backup=Path(temporary)/'previous'
        if workspace.exists():workspace.replace(backup)
        try:stage.replace(workspace)
        except BaseException:
            if backup.exists():backup.replace(workspace)
            raise
    return document
