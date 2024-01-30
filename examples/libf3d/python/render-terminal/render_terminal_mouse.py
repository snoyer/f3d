import fcntl
import os
from pathlib import Path
import re
import subprocess
import sys
from contextlib import contextmanager

import f3d


TEST_DATA_DIR = Path(__file__).parent.parent.parent.parent.parent / "testing/data"


def main():
    model_path = TEST_DATA_DIR / "suzanne.obj"
    options = {
        "scene.up-direction": "+y",
        "render.effect.tone-mapping": True,
        "render.effect.ambient-occlusion": True,
        "render.effect.translucency-support": True,
        "render.effect.anti-aliasing": True,
    }

    try:
        cols, rows = os.get_terminal_size()
    except OSError:
        cols, rows = 40, 20

    engine = f3d.Engine(f3d.Window.NATIVE_OFFSCREEN)
    engine.options.update(options)
    engine.loader.load_geometry(str(model_path))
    engine.window.size = cols, rows * 2
    engine.window.camera.reset_to_bounds()

    with mouse_capture():
        prev_drag_xy = None
        try:
            while True:
                for evt in pull_mouse_events():
                    try:
                        drag_x, drag_y = evt["btn1_drag_x"], evt["btn1_drag_y"]
                        if prev_drag_xy:
                            x0, y0 = prev_drag_xy
                            engine.window.camera.azimuth((drag_x - x0) * -1)
                            engine.window.camera.elevation((drag_y - y0) * 2)
                        prev_drag_xy = drag_x, drag_y
                    except KeyError:
                        prev_drag_xy = None

                img = engine.window.render_to_image()
                sys.stdout.write("\x1b[2J")
                sys.stdout.write(img.to_terminal_text())
        except KeyboardInterrupt:
            pass


EVENT_PATTERN = rb"""
\x1b\[(
    (\<35;(?P<move_x>\d+);(?P<move_y>\d+))
    |(\<32;(?P<btn1_drag_x>\d+);(?P<btn1_drag_y>\d+))
    |(\<33;(?P<btn2_drag_x>\d+);(?P<btn2_drag_y>\d+))
    |(\<34;(?P<btn3_drag_x>\d+);(?P<btn3_drag_y>\d+))
    |(\<64;(?P<scroll_up_x>\d+);(?P<scroll_up_y>\d+))
    |(\<65;(?P<scroll_dn_x>\d+);(?P<scroll_dn_y>\d+))
    |(\<0;(?P<btn1_up_x>\d+);(?P<btn1_up_y>\d+)m)
    |(\<0;(?P<btn1_dn_x>\d+);(?P<btn1_dn_y>\d+)M)
    |(\<1;(?P<btn2_up_x>\d+);(?P<btn2_up_y>\d+)m)
    |(\<1;(?P<btn2_dn_x>\d+);(?P<btn2_dn_y>\d+)M)
    |(\<2;(?P<btn3_up_x>\d+);(?P<btn3_up_y>\d+)m)
    |(\<2;(?P<btn3_dn_x>\d+);(?P<btn3_dn_y>\d+)M)
)
"""
EVENT_REGEX = re.compile(EVENT_PATTERN, re.VERBOSE)


def parse_mouse_events(data: bytes):
    for chunk in EVENT_REGEX.finditer(data):
        yield {k: int(v) for k, v in chunk.groupdict().items() if v}


def pull_mouse_events():
    with non_blocking_stdin() as stdin:
        if data := stdin.read():
            yield from parse_mouse_events(data)


@contextmanager
def non_blocking_stdin():
    fl_state = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
    fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, fl_state | os.O_NONBLOCK)

    yield sys.stdin.buffer

    fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, fl_state)


@contextmanager
def mouse_capture(full_reset: bool = True):
    # don't print events
    stty_state = subprocess.check_output(["stty", "-g"])
    os.system("stty -echo -icanon")

    # capture mouse events
    # https://invisible-island.net/xterm/ctlseqs/ctlseqs.html#h3-Any-event-tracking
    SET_VT200_MOUSE = "\x1b[?1000h"
    SET_ANY_EVENT_MOUSE = "\x1b[?1003h"
    SET_SGR_EXT_MODE_MOUSE = "\x1b[?1006h"
    sys.stdout.write(SET_ANY_EVENT_MOUSE)
    sys.stdout.write(SET_SGR_EXT_MODE_MOUSE)
    sys.stdout.flush()

    yield

    # restore stty
    subprocess.check_output(["stty", stty_state.decode().strip()])

    # reset terminal
    sys.stdout.write(SET_VT200_MOUSE)
    if full_reset:
        sys.stdout.write("\x1bc")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
