"""Application assembly, three independent time axes, and ordered hardware release."""
import time
import cv2
import numpy as np
from .camera_source import to_gray
from .rate_meter import FrameRate
from .rectangle_center import draw_result
from .laser_spot import draw_laser_overlay, PANEL_H
from .preview import display_result
from .telemetry import format_perf_line, format_slow_loop, MAIN_LOOP_SLOW_MS

_active = []


def _assert_idle():
    live = [r for r in _active if getattr(r, 'thread', None) is not None and r.thread.is_alive()]
    _active[:] = live
    if live:
        raise RuntimeError('previous worker/display/IMU thread is still running')


def _configure_touch(cfg, preview, touch_class, clock):
    if not preview.enabled or not getattr(cfg, 'ENABLE_TOUCH_BUTTON', True):
        return None
    w,h = preview.width, preview.height
    margin = min(8,max(0,min(w,h)//8))
    gap = max(1,min(8,h//12))
    count = 2 if getattr(cfg,'ENABLE_SEARCH_BUTTON',True) else 1
    bw = min(max(70,int(w*.30)),w-2*margin)
    bh = min(max(34,int(h*.20)),(h-2*margin-gap*(count-1))//count)
    if bh < 3 or bw < 3:
        return None
    preview.add_button('selftest',w-bw-margin,margin,w-margin,margin+bh)
    if count == 2:
        y=margin+bh+gap
        preview.add_button('search',w-bw-margin,y,w-margin,y+bh)
    return touch_class(cfg,clock=clock)


def _draw_preview(cfg, preview, gray, result, laser, age, fps, status, now):
    view=draw_result(gray,result,preview.render_width,top_offset=PANEL_H if laser else 0)
    if laser is not None:
        draw_laser_overlay(view,laser,result,min(1.,preview.render_width/float(gray.shape[1])),use_raw=cfg.AIM_USE_RAW)
    # Keep the performance line above Preview's bottom status line.
    cv2.putText(view,'disp %.1f low1 %.1f det %.1f age %.0fms' % (preview.fps,preview.low1,fps,age*1000),
                (8,view.shape[0]-30),cv2.FONT_HERSHEY_SIMPLEX,.40,(0,255,255),1)
    preview.render(view,now,status=status)


def _capture_loop(cfg, clock, app, image, state, build_id):
    cam,worker,controller,preview,touch = (state[k] for k in ('camera','worker','controller','preview','touch'))
    capture_rate=FrameRate()
    last_log=clock.perf_counter()
    read_times,draw_times,loop_times=[],[],[]
    while not app.need_exit():
        started=clock.perf_counter()
        img=cam.read()
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        if img is None:
            if not cfg.VISION_ASYNC:
                controller.idle()
            continue
        captured=clock.perf_counter()
        read_times.append((captured-started)*1000)
        if getattr(cfg,'LENS_CORR',False):
            img.lens_corr(strength=cfg.LENS_STRENGTH)
        gray=to_gray(img,image)
        capture_rate.record(captured)
        button=touch.poll(preview) if touch is not None else None
        if button == 'selftest': controller.request_test.set()
        elif button == 'search': controller.request_search.set()
        worker.submit(gray,captured)
        now=clock.perf_counter()
        snapshot,fps,_=worker.snapshot(now)
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        result,laser,age=display_result(snapshot,now,cfg.RESULT_MAX_AGE_S)
        if preview.due(now):
            t_draw=clock.perf_counter()
            try:
                _draw_preview(cfg,preview,gray,result,laser,age,fps,snapshot['status'] if snapshot else 'waiting',now)
            except Exception as exc:
                # Drawing/conversion errors belong to display, never to motor control.
                preview.enabled=False
                print('[preview] render disabled: %s' % exc)
            draw_times.append((clock.perf_counter()-t_draw)*1000)
        loop_times.append((clock.perf_counter()-started)*1000)
        now=clock.perf_counter()
        if now-last_log >= cfg.LOG_INTERVAL:
            capture_fps,_=capture_rate.values(now)
            display_fps,low1=preview.rate.values(now)
            avg,p95,worst,n=preview.take_show_stats()
            mean=lambda values: float(np.mean(values)) if values else 0.
            loop_avg=mean(loop_times)
            detector_result=snapshot.get('detector_result',snapshot['result']) if snapshot else {}
            print(format_perf_line(build_id,capture_fps,display_fps,low1,fps,result,
                mean(read_times),mean(draw_times),loop_avg,avg,p95,worst,n,age,
                preview.shown,preview.dropped,worker.dropped,detector_result=detector_result))
            if state['aim'] is not None: print(state['aim'].status_line())
            if p95 > 100: preview.warn_slow_once(p95)
            if loop_avg > MAIN_LOOP_SLOW_MS: print(format_slow_loop(loop_avg))
            last_log=now
            read_times.clear(); draw_times.clear(); loop_times.clear()


def _release(state):
    errors=[]
    def close(resource):
        if resource is not None and hasattr(resource,'close'):
            try: resource.close()
            except Exception as exc: errors.append(exc)
    worker=state.get('worker')
    close(worker)
    if worker is None or worker.thread is None or not worker.thread.is_alive():
        link=state.get('link')
        if link is not None:
            try:
                if link.stop() is False: errors.append(RuntimeError('stop on exit failed'))
            except Exception as exc: errors.append(exc)
        # Controller owns heading only after successful construction.
        if state.get('controller') is not None: close(state['controller'])
        else: close(state.get('heading'))
        close(link)
        close(state.get('uart'))
    close(state.get('preview'))
    touch=state.get('touch')
    close(touch.ts if touch is not None else None)
    close(state.get('camera'))
    if errors: raise RuntimeError('cleanup: '+'; '.join(map(str,errors)))


def run(entry, build_id=None):
    _assert_idle()
    cfg,clock=entry.Config,entry.time
    state=dict(camera=None,link=None,aim=None,uart=None,heading=None,controller=None,worker=None,preview=None,touch=None)
    try:
        from maix import app,camera,image
        cv2.setNumThreads(cfg.OPENCV_THREADS)
        cam=state['camera']=camera.Camera(cfg.CAMERA_W,cfg.CAMERA_H,image.Format.FMT_GRAYSCALE,fps=cfg.CAMERA_FPS,buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        calibrator=None
        if cfg.ENABLE_GIMBAL:
            from .stepper_link import StepperLink
            link=state['link']=StepperLink(cfg,clock=clock)
            aim=state['aim']=entry.GimbalAim(cfg)
            if link.enable() is False: aim.fault('enable_failed')
            calibration_cfg=type('YawCalibration',(cfg,),{'CALIBRATE_AXIS':'yaw'}) if cfg.CALIBRATE_AXIS=='auto' else cfg
            calibrator=entry.GimbalCalibrator(calibration_cfg,link,clock.perf_counter())
            if calibrator.done: calibrator=None
            info=entry.probe_imu()
            print('[imu] %s' % ('available' if info['available'] else 'unavailable: '+str(info['error'])))
            print('[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d' % (cfg.YAW_DEV,cfg.ADDR_YAW,cfg.PITCH_DEV,cfg.ADDR_PITCH,cfg.ACC_TRACK))
        elif cfg.ENABLE_UART:
            from maix import uart
            state['uart']=uart.UART(cfg.UART_PORT,cfg.UART_BAUD)
        preview=state['preview']=entry.Preview(cfg,clock=clock)
        _active.append(preview)
        state['touch']=_configure_touch(cfg,preview,entry.TouchButtons,clock)
        if getattr(cfg,'ENABLE_IMU_FEEDFORWARD',False) or (getattr(cfg,'ENABLE_SEARCH_BUTTON',True) and getattr(cfg,'SEARCH_USE_IMU',True)):
            from .imu_heading import HeadingFeedforward
            state['heading']=HeadingFeedforward(cfg,clock=clock)
            _active.append(state['heading'])
        controller=state['controller']=entry.VisionControl(cfg,state['link'],state['aim'],calibrator,state['uart'],clock=clock,heading=state['heading'])
        if cfg.CALIBRATE_AXIS=='auto': controller._auto_axis='yaw'
        worker=state['worker']=entry.LatestFrameWorker(controller.process,controller.idle,cfg.VISION_ASYNC,clock=clock)
        _active.append(worker)
        print('[rect-center] %s search_width=%d display_width=%d display_fps=%d' % (build_id or entry.BUILD_ID,cfg.SEARCH_WIDTH,preview.render_width,cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC,preview.render_width,preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC,cfg.RESULT_MAX_AGE_S*1000))
        print('[ui] 上电自检已改为屏幕按钮：SELF-TEST / SEARCH')
        _capture_loop(cfg,clock,app,image,state,build_id or entry.BUILD_ID)
    finally:
        _release(state)
