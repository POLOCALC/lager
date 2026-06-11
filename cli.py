import time
import parameters as params

from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text




def build_layout():
    layout = Layout()
    layout.split(
        Layout(name="main"),
        Layout(name="footer", size=params.LOG_LINES_TO_SHOW + 2)
    )
    layout["main"].split_row(
        Layout(name="drone", ratio=3),
        Layout(name="gimbal", ratio=1)
    )
    layout["footer"].split_row(
        Layout(name="logs", ratio=3),
        Layout(name="poi", ratio=1)
    )
    return layout

def get_logs_from_file(logfile, n=params.LOG_LINES_TO_SHOW):
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
    
def log_panel(logs, lines=params.LOG_LINES_TO_SHOW):
    log_text = Text()
    for log in logs[-lines:]:
        line = Text(log + "\n")
        # highlight log levels with colors
        line.highlight_words(["INFO"], "bold green")
        line.highlight_words(["WARNING"], "bold yellow")
        line.highlight_words(["ERROR"], "bold red")
        log_text.append(line)
    return Panel(log_text, title="[bold yellow]Most recent logs", border_style="yellow", title_align="left")

def poi_panel(poi_data):
    if poi_data is None:
        return Panel(Text("No point of interest data available"), title="[bold cyan]Point of interest", border_style="cyan", title_align="left")
    
    table = Table.grid(expand=True, padding=(0, -5))
    table.add_column(justify="left", style="bold yellow", no_wrap=False)
    table.add_column(justify="left", no_wrap=True)
    for k, v in poi_data.items():
        table.add_row(k, str(v))
    return Panel(table, title="[bold cyan]Point of interest", border_style="cyan", title_align="left")

def live_display(drone_panel=None, gimbal_panel=None, logfile=None, poi_data=None, refresh_rate=1):
    layout = build_layout()
    with Live(layout, refresh_per_second=refresh_rate, screen=True):
        while True:
            try:
                if drone_panel is not None:
                    layout["drone"].update(drone_panel())
                if gimbal_panel is not None:
                    layout["gimbal"].update(gimbal_panel())
                if logfile is not None:
                    layout["logs"].update(log_panel(get_logs_from_file(logfile), lines=params.LOG_LINES_TO_SHOW))
                if poi_data is not None:
                    layout["poi"].update(poi_panel(poi_data()))
            except Exception:
                pass  # never let a transient panel error crash the display thread
            time.sleep(1/refresh_rate)
