"""One-command PC checks. Missing hardware/fixtures are explicit, never fake passes."""
import argparse
import subprocess
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
TESTS=(
 'test_performance.py','test_kalman.py','test_real_scene.py','test_target_api.py',
 'test_tracking.py','test_runtime_performance.py','test_gimbal_aim.py','test_gimbal_tracking.py',
 'test_aimloop_tracking.py','test_gimbal_search.py','test_imu_heading.py','test_imu_turn_tracking.py',
 'test_preview_stream.py','test_app_runtime.py','test_main_self_test.py','test_extension_control.py',
 'test_extension_review.py','test_laser.py','test_images_e2e.py')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quick',action='store_true',help='Skip only long image and runtime integration suites')
    args=parser.parse_args()
    commands=[['tools/build_single.py','--check'],['tools/build_single.py','--smoke'],['tools/check_app.py','--manifest']]
    for test in TESTS:
        if args.quick and test in ('test_images_e2e.py','test_main_self_test.py'):
            print('SKIP quick:',test,flush=True)
        else: commands.append(['tools/'+test])
    failed=[]
    for command in commands:
        print('\nCHECK '+' '.join(command),flush=True)
        try: result=subprocess.run([sys.executable,'-B',*command],cwd=ROOT,timeout=180)
        except subprocess.TimeoutExpired:
            failed.append(command); print('FAIL timeout',flush=True); continue
        if result.returncode: failed.append(command)
    print('\nPC checks: %d passed / %d failed'%(len(commands)-len(failed),len(failed)))
    for command in failed: print('FAIL '+' '.join(command))
    return bool(failed)


if __name__=='__main__': sys.exit(main())
