from PIL import Image, ImageDraw, ImageFont
import imageio


COLOR_STATIC = (102, 102, 102)
COLOR_STATIC_EDGE = (51, 51, 51)
COLOR_MOVING = (68, 136, 255)
COLOR_MOVING_EDGE = (34, 85, 204)
COLOR_ROBOT = (255, 51, 51)
COLOR_ROBOT_EDGE = (204, 0, 0)
COLOR_PATH = (34, 204, 34)
COLOR_BG = (255, 255, 255)
COLOR_GRID = (200, 200, 200)
COLOR_DEST = (204, 0, 0)

COLOR_PANEL_BG = (240, 240, 240)
COLOR_PANEL_BORDER = (100, 100, 100)
COLOR_TITLE = (40, 40, 40)
COLOR_LABEL = (80, 80, 80)
COLOR_VALUE = (20, 20, 20)
COLOR_STATUS_OK = (0, 150, 0)
COLOR_STATUS_WAIT = (180, 120, 0)
COLOR_STATUS_TIMEOUT = (180, 0, 0)


class SimulationVisualizer:
    def __init__(self, m_l=100, m_b=100, scale=5, panel_width=200):
        self.m_l = m_l
        self.m_b = m_b
        self.scale = scale
        self.img_w = int(m_l * scale)
        self.img_h = int(m_b * scale)
        self.panel_width = panel_width
        self.canvas_w = self.img_w + self.panel_width
        self.canvas_h = self.img_h
        self.sim_bg = None
        self._load_fonts()

    def _load_fonts(self):
        self.font_title = None
        self.font_data = None
        self.font_small = None
        for path, size in [("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14),
                           ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)]:
            try:
                self.font_title = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
                self.font_data = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
                self.font_small = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
                return
            except (OSError, IOError):
                pass
        self.font_title = ImageFont.load_default()
        self.font_data = ImageFont.load_default()
        self.font_small = ImageFont.load_default()

    def _to_px(self, x, y):
        return (x * self.scale, (self.m_b - y) * self.scale)

    def _to_px_rect(self, rect):
        x1, y1 = self._to_px(rect.x, rect.y + rect.h)
        x2, y2 = self._to_px(rect.x + rect.w, rect.y)
        return [x1, y1, x2, y2]

    def setup_figure(self):
        self.sim_bg = Image.new('RGB', (self.img_w, self.img_h), COLOR_BG)
        bg_draw = ImageDraw.Draw(self.sim_bg)
        for i in range(0, self.img_w, self.scale):
            bg_draw.line([(i, 0), (i, self.img_h)], fill=COLOR_GRID, width=1)
        for j in range(0, self.img_h, self.scale):
            bg_draw.line([(0, j), (self.img_w, j)], fill=COLOR_GRID, width=1)

    def _draw_panel(self, draw, state):
        px = self.img_w
        w = self.panel_width

        draw.rectangle([px, 0, px + w, self.canvas_h], fill=COLOR_PANEL_BG)
        draw.rectangle([px, 0, px + w, self.canvas_h], outline=COLOR_PANEL_BORDER, width=2)

        cx = px + w // 2
        title = "SIMULATION INFO"
        tb = draw.textbbox((0, 0), title, font=self.font_title)
        draw.text((cx - (tb[2] - tb[0]) // 2, 12), title,
                  fill=COLOR_TITLE, font=self.font_title)

        sep_y = 38
        draw.line([(px + 10, sep_y), (px + w - 10, sep_y)],
                  fill=COLOR_PANEL_BORDER, width=1)

        rows = [
            ("Time:", f"{state['time_elapsed']:.2f}s"),
            ("Steps:", str(state['tick'])),
            ("Re-plans:", str(state['replan_count'])),
            ("Seed:", str(state['seed'])),
        ]

        ry = 50
        for label, value in rows:
            draw.text((px + 14, ry), label, fill=COLOR_LABEL, font=self.font_data)
            draw.text((px + 100, ry), value, fill=COLOR_VALUE, font=self.font_data)
            ry += 24

        ry += 6
        draw.line([(px + 10, ry), (px + w - 10, ry)], fill=COLOR_PANEL_BORDER, width=1)
        ry += 10

        status_label = "Status:"
        draw.text((px + 14, ry), status_label, fill=COLOR_LABEL, font=self.font_data)

        if state['arrived']:
            status_text = "ARRIVED"
            status_color = COLOR_STATUS_OK
        elif state['done']:
            status_text = "TIMEOUT"
            status_color = COLOR_STATUS_TIMEOUT
        else:
            status_text = "IN PROGRESS"
            status_color = COLOR_STATUS_WAIT
        slx = px + 100
        draw.text((slx, ry), status_text, fill=status_color, font=self.font_data)

    def render(self, state):
        canvas = Image.new('RGB', (self.canvas_w, self.canvas_h), COLOR_PANEL_BG)

        sim_img = self.sim_bg.copy()
        draw = ImageDraw.Draw(sim_img)

        for obs in state['static_obstacles']:
            coords = self._to_px_rect(obs)
            draw.rectangle(coords, fill=COLOR_STATIC, outline=COLOR_STATIC_EDGE, width=1)

        for mo in state['moving_obstacles']:
            coords = self._to_px_rect(mo.rect)
            draw.rectangle(coords, fill=COLOR_MOVING, outline=COLOR_MOVING_EDGE, width=1)

        rr = state['robot_rect']
        coords = self._to_px_rect(rr)
        draw.rectangle(coords, fill=COLOR_ROBOT, outline=COLOR_ROBOT_EDGE, width=2)

        path = state['path']
        if path and len(path) > 1:
            for i in range(len(path) - 1):
                x1, y1 = self._to_px(path[i][0], path[i][1])
                x2, y2 = self._to_px(path[i + 1][0], path[i + 1][1])
                draw.line([(x1, y1), (x2, y2)], fill=COLOR_PATH, width=2)

        dx, dy = self._to_px(state['robot_dest'][0], state['robot_dest'][1])
        s = 4
        draw.line([(dx - s, dy - s), (dx + s, dy + s)], fill=COLOR_DEST, width=2)
        draw.line([(dx + s, dy - s), (dx - s, dy + s)], fill=COLOR_DEST, width=2)

        rx, ry = self._to_px(state['robot_pos'][0], state['robot_pos'][1])
        draw.ellipse([rx - 2, ry - 2, rx + 2, ry + 2], fill=COLOR_ROBOT)

        canvas.paste(sim_img, (0, 0))

        panel_draw = ImageDraw.Draw(canvas)
        self._draw_panel(panel_draw, state)

        return canvas

    def capture_frame(self, pil_image):
        return pil_image

    def close(self):
        self.sim_bg = None


def save_gif(frames, filepath, duration=0.15):
    if not frames:
        return False
    imageio.mimsave(filepath, [f for f in frames], duration=duration, loop=0)
    return True
