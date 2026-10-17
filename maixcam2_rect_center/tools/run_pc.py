"""Run the actual detector on Unicode image paths, video, or a camera."""
import argparse
import json
import sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from target_api import CenterTracker, load_config
from vision.rectangle_center import draw_result


def imread_unicode(path,flags=cv2.IMREAD_GRAYSCALE):
    try: return cv2.imdecode(np.fromfile(str(path),np.uint8),flags)
    except (OSError,cv2.error): return None


def imwrite_unicode(path,image):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    ok,data=cv2.imencode(path.suffix or '.png',image)
    if not ok: raise OSError('image encoding failed: '+str(path))
    data.tofile(str(path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--images',nargs='+')
    source.add_argument('--video')
    source.add_argument('--camera',type=int)
    parser.add_argument('--show',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--limit',type=int,default=0)
    args=parser.parse_args()
    tracker=CenterTracker(load_config(),with_laser=True)
    cv2.setNumThreads(tracker.cfg.OPENCV_THREADS)
    def process(gray,name,index):
        if gray is None: raise OSError('cannot read '+name)
        result=tracker.track(gray)
        print(json.dumps(dict(source=name,valid=result.valid,center=result.center,stage=result.stage,
            ms=result.ms,masks=result.get('masks'),dx=result.dx,dy=result.dy),ensure_ascii=False))
        if args.show or args.output:
            canvas=draw_result(gray,result,gray.shape[1])
            if args.output: imwrite_unicode(args.output/('%04d.png'%index),canvas)
            if args.show:
                cv2.imshow('Target Center',canvas)
                return cv2.waitKey(1)&255 != 27
        return True
    if args.images:
        files=[]
        for item in args.images:
            path=Path(item)
            files.extend(sorted(p for p in path.rglob('*') if p.suffix.lower() in ('.png','.jpg','.jpeg','.bmp')) if path.is_dir() else [path])
        if not files: raise SystemExit('No input images')
        for i,path in enumerate(files):
            tracker.reset()  # Independent files must not borrow another scene's lock.
            if not process(imread_unicode(path),str(path),i): break
    else:
        cap=cv2.VideoCapture(args.video if args.video else args.camera)
        try:
            if not cap.isOpened(): raise OSError('cannot open video/camera')
            index=0
            while not args.limit or index<args.limit:
                ok,frame=cap.read()
                if not ok: break
                if not process(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),str(index),index): break
                index+=1
        finally: cap.release()
    if args.show: cv2.destroyAllWindows()


if __name__=='__main__': main()
