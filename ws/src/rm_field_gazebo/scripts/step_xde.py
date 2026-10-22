#!/usr/bin/env python3
"""Extract STEP/XDE assembly geometry for the offline field importer.

Dependencies: requirements-cad.txt. Not imported by ROS/Gazebo at runtime.
Coordinates in this cache are millimetres; assembly transforms and per-face
colours are retained. The official-field generator converts these to ENU metres.
The cache belongs in a temporary/work directory, never beside runtime meshes.
"""
import os,sys,time,json,collections,math,hashlib,argparse
from pathlib import Path
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString,TCollection_AsciiString
from OCP.TDF import TDF_Label,TDF_LabelSequence,TDF_Tool
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ColorType
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED,TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.Quantity import Quantity_Color
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('source',type=Path)
parser.add_argument('cache',type=Path)
parser.add_argument('--deflection-mm',type=float,default=1.5)
args=parser.parse_args()
if args.deflection_mm<=0:parser.error('deflection must be positive')
src=args.source.resolve();out=args.cache.resolve();out.mkdir(parents=True,exist_ok=True)
start=time.monotonic();print('READ',src.name,flush=True)
reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(True);reader.SetLayerMode(True)
status=reader.ReadFile(str(src));print('READ_DONE',status,round(time.monotonic()-start,1),flush=True)
doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
assert reader.Transfer(doc),'STEP transfer failed'
print('TRANSFER_DONE',round(time.monotonic()-start,1),flush=True)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());ct=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
def entry(label):
 s=TCollection_AsciiString();TDF_Tool.Entry_s(label,s);return s.ToCString()
def name(label):
 attr=TDataStd_Name()
 return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else entry(label)
def rgb(obj,fallback=(.65,.65,.65)):
 color=Quantity_Color()
 for kind in [XCAFDoc_ColorType.XCAFDoc_ColorSurf,XCAFDoc_ColorType.XCAFDoc_ColorGen]:
  if (ct.GetColor_s(obj,kind,color) if isinstance(obj,TDF_Label) else ct.GetColor(obj,kind,color)):return (color.Red(),color.Green(),color.Blue())
 return fallback
def bbox(shape):
 b=Bnd_Box();BRepBndLib.Add_s(shape,b,False)
 return None if b.IsVoid() else list(b.Get())
def matrix(loc):
 tr=loc.Transformation();return [[tr.Value(r,c) for c in range(1,5)] for r in range(1,4)]+[[0,0,0,1]]
seq=TDF_LabelSequence();st.GetFreeShapes(seq);instances=[];definitions={};tree=[]
def walk(label,loc,path):
 isref=st.IsReference_s(label);ref=TDF_Label()
 actual=label
 if isref:
  assert st.GetReferredShape_s(label,ref)
  actual=ref;loc=loc.Multiplied(st.GetLocation_s(label))
 eid=entry(actual);n=name(actual);path=path+[n]
 parts=TDF_LabelSequence();st.GetComponents_s(actual,parts,False)
 shape=st.GetShape_s(actual)
 box=bbox(shape.Moved(loc)) if not shape.IsNull() else None
 tree.append(dict(path=path,entry=eid,box=box,children=parts.Length(),transform=matrix(loc)))
 if parts.Length():
  for i in range(1,parts.Length()+1):walk(parts.Value(i),loc,path)
 else:
  definitions[eid]=(actual,shape)
  instances.append(dict(path=path,entry=eid,box=box,transform=matrix(loc),color=rgb(label,rgb(actual))))
for i in range(1,seq.Length()+1):walk(seq.Value(i),TopLoc_Location(),[])
digest=hashlib.sha256()
with src.open('rb') as stream:
 for block in iter(lambda:stream.read(8*1024*1024),b''):digest.update(block)
metadata=dict(source=src.name,source_sha256=digest.hexdigest(),unit='mm',
 deflection_mm=args.deflection_mm,instances=instances,tree=tree,definitions=[])
(out/'metadata.json').write_text(json.dumps(metadata,indent=2));print('ASSEMBLY',len(tree),'instances',len(instances),'unique',len(definitions),flush=True)
for index,(eid,(label,shape)) in enumerate(definitions.items()):
 if shape.IsNull():continue
 b=bbox(shape);fallback=rgb(label);base=eid.replace(':','_')
 print('MESH',index+1,len(definitions),name(label),'bbox',b,flush=True)
 BRepMesh_IncrementalMesh(shape,args.deflection_mm,False,.3,False).Perform()
 xyz=[];tris=[];colors=[];planes=[];face_index=0
 ex=TopExp_Explorer(shape,TopAbs_FACE)
 while ex.More():
  face=TopoDS.Face_s(ex.Current());face_index+=1;col=rgb(face,fallback);loc=TopLoc_Location();mesh=BRep_Tool.Triangulation_s(face,loc)
  if mesh is not None and mesh.NbTriangles()>0:
   offset=len(xyz);tr=loc.Transformation()
   xyz.extend([(pt.X(),pt.Y(),pt.Z()) for pt in (mesh.Node(i).Transformed(tr) for i in range(1,mesh.NbNodes()+1))])
   for ti in range(1,mesh.NbTriangles()+1):
    a,b_,c=mesh.Triangle(ti).Get()
    if face.Orientation()==TopAbs_REVERSED:b_,c=c,b_
    tris.append((offset+a-1,offset+b_-1,offset+c-1));colors.append(col)
  surf=BRepAdaptor_Surface(face,True)
  if surf.GetType()==GeomAbs_Plane:
   prop=GProp_GProps();BRepGProp.SurfaceProperties_s(face,prop);area=prop.Mass()
   if 100<=area<=100000:
    center=prop.CentreOfMass();normal=surf.Plane().Axis().Direction()
    n=np.array([normal.X(),normal.Y(),normal.Z()])*(-1 if face.Orientation()==TopAbs_REVERSED else 1)
    planes.append(dict(face=face_index,area=area,center=[center.X(),center.Y(),center.Z()],normal=n.tolist(),box=bbox(face),color=col))
  ex.Next()
 vertices=np.asarray(xyz,dtype=np.float32);triangles=np.asarray(tris,dtype=np.int32);color=np.asarray(colors,dtype=np.float32)
 np.savez_compressed(out/(base+'.npz'),vertices=vertices,triangles=triangles,colors=color)
 record=dict(entry=eid,name=name(label),file=base+'.npz',box=b,vertices=len(vertices),triangles=len(triangles),planes=planes)
 metadata['definitions'].append(record)
 (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
 print('SAVED',base,vertices.shape,triangles.shape,round(time.monotonic()-start,1),flush=True)
metadata['complete']=True
(out/'metadata.json').write_text(json.dumps(metadata,indent=2))
print('DONE',round(time.monotonic()-start,1),flush=True)
