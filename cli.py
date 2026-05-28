import time

from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


LOG_LINES_TO_SHOW = 12

def build_layout():
    layout = Layout()
    layout.split(
        Layout(name="main"),
        Layout(name="footer", size=LOG_LINES_TO_SHOW + 2)
    )
    layout["main"].split_row(
        Layout(name="drone", ratio=3),
        Layout(name="gimbal", ratio=1)
    )
    return layout

def render_footer(logs, lines=LOG_LINES_TO_SHOW):
    log_text = Text()
    for log in logs[-lines:]:
        line = Text(log + "\n")
        # highlight log levels with colors
        line.highlight_words(["INFO"], "bold green")
        line.highlight_words(["WARNING"], "bold yellow")
        line.highlight_words(["ERROR"], "bold red")
        log_text.append(line)
    return Panel(log_text, title="[bold yellow]Most recent logs", border_style="yellow", title_align="left")

def live_display(drone_panel=None, gimbal_panel=None, logfile=None, refresh_rate=1):
    layout = build_layout()
    with Live(layout, refresh_per_second=refresh_rate, screen=True):
        while True:
            if drone_panel is not None:
                layout["drone"].update(drone_panel())
            if gimbal_panel is not None:
                layout["gimbal"].update(gimbal_panel())
            if logfile is not None:
                layout["footer"].update(render_footer(get_logs_from_file(logfile), lines=LOG_LINES_TO_SHOW))
            # Add sleep or event wait as needed
            time.sleep(1/refresh_rate)

def get_logs_from_file(logfile, n=LOG_LINES_TO_SHOW):
    """Return the last n lines from the given log file."""
    try:
        with open(logfile, 'rb') as f:
            f.seek(0, 2)
            filesize = f.tell()
            size = 1024
            data = b''
            while filesize > 0 and data.count(b'\n') <= n:
                read_size = min(size, filesize)
                f.seek(filesize - read_size)
                data = f.read(read_size) + data
                filesize -= read_size
            return data.decode(errors='replace').splitlines()[-n:]
    except Exception as e:
        return [f"Log read error: {e}"]