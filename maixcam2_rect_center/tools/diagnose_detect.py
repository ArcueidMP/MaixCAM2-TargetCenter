"""Print per-side dark-band evidence; --front writes the independent front view."""
import argparse
import json
from pathlib import Path
from run_pc import imread_unicode,imwrite_unicode,load_config
from vision.rectangle_center import RectangleCenterDetector


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    parser.add_argument('--front',type=Path)
    args=parser.parse_args()
    gray=imread_unicode(args.image)
    if gray is None: raise SystemExit('Cannot read image: '+args.image)
    detector=RectangleCenterDetector(load_config(FRONT_VIEW_SHORT_PX=512 if args.front else 0))
    for level in (0,1):
        proposals=detector.locator.locate(gray,level=level)
        print('level=%d candidates=%d stats=%s'%(level,len(proposals),detector.locator.last_stats))
        for i,(quad,kind) in enumerate(proposals):
            candidate=detector._verify(gray,quad,kind,diagnostic=True)
            print(json.dumps(dict(candidate=i,kind=kind,accepted=candidate is not None,
                evidence=detector.last_diagnostic),ensure_ascii=False))
    result=detector.process(gray)
    print('result valid=%s stage=%s masks=%d ms=%.2f'%(result['valid'],result['stage'],result['masks'],result['detector_ms']))
    if args.front and result.get('front_view') is not None:
        imwrite_unicode(args.front,result['front_view'])
        print('front independent center=%s error=%s consistent=%s'%(result.get('front_view_center'),result.get('front_view_error_px'),result.get('geometry_consistent')))


if __name__=='__main__': main()
