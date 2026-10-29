# SPDX-License-Identifier: Apache-2.0
"""「AI の視点」パネルの描画：真上カメラ画像に物体 ID・判断結果を重ね、
下に指示文と AI の判断・進行状況を日本語で表示する。動画の説明用。ROS 非依存。"""

import glob
import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover
    Image = None

STATE_COLORS = {          # RGB
    'pending': (255, 255, 255),
    'keep': (60, 200, 90),
    'discard': (235, 70, 60),
    'leave': (90, 170, 255),
    'active': (255, 190, 0),
    'done': (150, 150, 150),
}


def project(scene, x, y, z=0.0):
    """土台基準の点を真上カメラの画素座標へ。画像の上=ロボット正面(+x)、左=+y。"""
    cam = scene.camera
    w, h = cam['width'], cam['height_px']
    f = (w / 2.0) / math.tan(cam['hfov'] / 2.0)
    depth = cam['height'] - z
    u = w / 2.0 - f * (y - cam['y']) / depth
    v = h / 2.0 - f * (x - cam['x']) / depth
    return int(round(u)), int(round(v))


def _find_font():
    pats = ['/usr/share/fonts/**/NotoSansCJK*.tt*', '/usr/share/fonts/**/*Gothic*.tt*',
            '/usr/share/fonts/**/fonts-japanese-gothic.ttf', '/usr/share/fonts/**/*IPA*.tt*',
            '/usr/share/fonts/**/Takao*.tt*', '/usr/share/fonts/**/*CJK*.tt*']
    for p in pats:
        hits = sorted(glob.glob(p, recursive=True))
        if hits:
            return hits[0]
    return None


class Panel:

    def __init__(self, scene):
        self.scene = scene
        self.font_path = _find_font() if Image else None
        self._fonts = {}

    def font(self, size):
        if size not in self._fonts:
            self._fonts[size] = (ImageFont.truetype(self.font_path, size)
                                 if self.font_path else ImageFont.load_default())
        return self._fonts[size]

    def wrap(self, lines, max_w):
        """パネル幅に収まるように長い行を折り返す。"""
        if Image is None:
            return lines
        out = []
        for i, line in enumerate(lines):
            f = self.font(22 if i == 0 else 20)
            cur = ''
            for ch in line:
                if cur and f.getlength(cur + ch) > max_w:
                    out.append(cur)
                    cur = '  ' + ch
                else:
                    cur += ch
            out.append(cur)
            if i == 0:
                self._first_lines = len(out)
        return out

    def render(self, image_rgb, objects, lines, active_id=None):
        """image_rgb: HxWx3 uint8（None なら灰色の画面）。lines: 下段に出す文字列。"""
        cam = self.scene.camera
        w, h = cam['width'], cam['height_px']
        if image_rgb is None:
            image_rgb = np.full((h, w, 3), 70, np.uint8)
        img = np.ascontiguousarray(image_rgb[:, :, :3]).copy()
        lines = self.wrap(lines, img.shape[1] - 28)
        text_h = 34 + 30 * max(len(lines), 1)
        canvas = np.full((img.shape[0] + text_h, img.shape[1], 3), 25, np.uint8)
        canvas[:img.shape[0]] = img

        if Image is None:
            return canvas
        pil = Image.fromarray(canvas)
        d = ImageDraw.Draw(pil)
        for o in objects:
            u, v = project(self.scene, o.x, o.y, o.height)
            state = 'active' if o.id == active_id else o.state
            if state == 'done' and o.category == 'trash':
                continue  # ゴミ箱に入れた物は表示しない
            col = STATE_COLORS.get(state, (255, 255, 255))
            rad = max(14, int(0.5 * o.footprint * w / (2 * math.tan(cam['hfov'] / 2) *
                                                        cam['height'])))
            d.ellipse([u - rad, v - rad, u + rad, v + rad], outline=col, width=3)
            tag = o.id.replace('obj', '#')
            if o.state in ('keep', 'discard', 'leave', 'done') and o.category:
                tag += ' ' + {'trash': 'ゴミ', 'stay': 'そのまま'}.get(o.category, '残す')
            d.text((u + rad + 3, v - 12), tag, fill=col, font=self.font(18),
                   stroke_width=2, stroke_fill=(0, 0, 0))
        y = img.shape[0] + 10
        for i, line in enumerate(lines):
            head = i < getattr(self, '_first_lines', 1)
            d.text((14, y), line, fill=(255, 210, 80) if head else (255, 255, 255),
                   font=self.font(22 if head else 20))
            y += 30
        return np.asarray(pil)


def encode_jpeg(image_rgb, quality=85):
    if cv2 is None or image_rgb is None:
        return None
    ok, buf = cv2.imencode('.jpg', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR),
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


def annotate_for_vlm(scene, image_rgb):
    """VLM に渡す画像：物体 ID だけを重ねる（判断の手がかりを与えすぎない）。"""
    if image_rgb is None or Image is None:
        return image_rgb
    panel = Panel(scene)
    pil = Image.fromarray(np.ascontiguousarray(image_rgb[:, :, :3]))
    d = ImageDraw.Draw(pil)
    for o in scene.objects:
        u, v = project(scene, o.x, o.y, o.height)
        d.text((u + 12, v - 22), o.id, fill=(255, 255, 0), font=panel.font(18),
               stroke_width=2, stroke_fill=(0, 0, 0))
    return np.asarray(pil)
