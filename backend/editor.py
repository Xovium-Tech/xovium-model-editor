#!/usr/bin/env python3
"""JSON command interface used by the model editor UI and automation."""
import argparse
import contextlib
import json
from pathlib import Path
import sys
from project import ProjectError, CATALOG, new_project, validate_project, load_workspace, write_workspace, import_project, save_project

class Parser(argparse.ArgumentParser):
    def error(self,message):raise ProjectError(message)

def parser():
    cli=Parser(description='Xovium portable model editor backend')
    commands=cli.add_subparsers(dest='command',required=True)
    new=commands.add_parser('new');new.add_argument('--workspace',required=True);new.add_argument('--name',default='Untitled model');new.add_argument('--vehicle',choices=('fixed-wing','rover','custom'),default='fixed-wing')
    load=commands.add_parser('import');load.add_argument('source');load.add_argument('--workspace',required=True);load.add_argument('--mesh-helper')
    save=commands.add_parser('save');save.add_argument('--workspace',required=True);save.add_argument('--output',required=True)
    export=commands.add_parser('export');export.add_argument('--workspace',required=True);export.add_argument('--target',choices=('gazebo','isaac','sih','unreal','flightgear'),required=True);export.add_argument('--output',required=True)
    check=commands.add_parser('validate');check.add_argument('--workspace',required=True)
    commands.add_parser('catalog')
    return cli

def main(argv=None):
    try:
        args=parser().parse_args(argv)
        result={'ok':True}
        with contextlib.redirect_stdout(sys.stderr):
            if args.command=='catalog':result['catalog']=CATALOG
            else:
                workspace=Path(args.workspace).absolute();result['workspace']=str(workspace)
                if args.command=='new':
                    if workspace.exists() and any(workspace.iterdir()):raise ProjectError('New project workspace must be empty')
                    document=new_project(args.name,args.vehicle);write_workspace(document,workspace)
                elif args.command=='import':document=import_project(args.source,workspace,args.mesh_helper)
                elif args.command=='save':
                    output=save_project(workspace,args.output);document=load_workspace(workspace);result['output']=str(output)
                elif args.command=='export':
                    from exports import export_project
                    document=load_workspace(workspace)
                    result['output']=str(Path(args.output).absolute())
                    result['target']=args.target
                    result['export']=export_project(document,workspace,args.target,Path(args.output))
                else:document=load_workspace(workspace)
                result['name']=document['name'];result['warnings']=document.get('warnings',[])
                result['counts']={key:len(document.get(key,[])) for key in ('links','visuals','sensors','plugins','joints')}
        print(json.dumps(result,allow_nan=False,default=str))
        return 0
    except Exception as error:
        print(json.dumps({'ok':False,'error':str(error)}))
        return 1

if __name__=='__main__':sys.exit(main())
