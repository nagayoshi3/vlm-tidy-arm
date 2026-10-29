# SPDX-License-Identifier: Apache-2.0
"""整理判断ノードの中身：VLM に机の画像と物体リストを渡し、
「残す/捨てる」「どこにまとめるか」「片付ける順番」を JSON で返させる。ROS 非依存。

プロバイダは環境変数で切り替える（API キーはコードやリポジトリに書かない）。
  TIDY_VLM_PROVIDER = anthropic | openai | gemini | mock
  ANTHROPIC_API_KEY / OPENAI_API_KEY / GEMINI_API_KEY
  TIDY_VLM_MODEL    = モデル名（省略時は各社の既定値）
mock は scene.yaml の正解を返すだけで、API キーなしの動作確認用。
"""

import base64
import json
import os
import re
import time
import urllib.error
import urllib.request

DEFAULT_MODELS = {
    'anthropic': 'claude-haiku-4-5-20251001',
    'openai': 'gpt-4o-mini',
    'gemini': 'gemini-2.5-flash',
}

KEY_ENVS = {
    'anthropic': 'ANTHROPIC_API_KEY',
    'openai': 'OPENAI_API_KEY',
    'gemini': 'GEMINI_API_KEY',
}


class JudgeError(Exception):
    pass


def direction_word(x, y):
    import math
    th = math.degrees(math.atan2(y, x))
    if th > 30:
        side = '左'
    elif th < -30:
        side = '右'
    else:
        side = '正面'
    dist = '奥' if math.hypot(x, y) > 0.125 else '手前'
    return f'{side}{dist}'


def build_prompt(scene, instruction):
    zone_lines = '\n'.join(
        f'  - {z.category}: {z.label}（置き場 {len(z.slots)} か所）'
        for z in scene.zones.values())
    obj_lines = '\n'.join(
        f'  - {o.id}: {o.name}（ロボットから見て{direction_word(o.x, o.y)}）'
        for o in scene.objects)
    cats = ' / '.join(scene.categories)
    return f"""あなたは机の上を整理整頓するロボットアームの頭脳です。
画像は机を真上から見たもので、画像の上側がロボットの正面（奥）、画像の左側がロボットから見て左です。
画像中の番号ラベルは下の物体 ID に対応します。

# ユーザーの指示
{instruction}

# 机の上の物（物体認識の結果）
{obj_lines}

# 片付け先
{zone_lines}
  - trash: ゴミ箱

# やること
1. 整理：指示の目的に照らして、各物を次の 3 つに分ける。
   - discard：要らない物。ゴミ箱に捨てる
   - keep：要るが今は使わない物。片付け先にまとめる
   - leave：指示の目的で今まさに使う物。動かさず手元に置いたままにする
2. 整頓：keep の物は、片付け先の category（{cats}）のどれにまとめるか決める。
   discard の category は trash、leave の category は stay にする。
3. 片付ける順番を決める（例：ゴミを先に捨てて作業スペースを空ける、など理由のある順番）。
   leave の物は動かさないが、order には含める。

# 出力
次の JSON だけを出力してください。説明文やコードブロックは不要です。
{{
  "scene_summary": "机の状態を一文で（40字以内）",
  "items": [
    {{"id": "obj1", "decision": "keep" / "discard" / "leave" のどれか,
      "category": "{cats} / trash / stay のどれか", "reason": "判断理由（30字以内）"}}
  ],
  "order": ["片付ける順に物体 ID を全部並べる"],
  "order_reason": "その順番にした理由（40字以内）"
}}
items と order には上の物体 ID をすべて 1 回ずつ含めてください。"""


def extract_json(text):
    text = text.strip()
    text = re.sub(r'^```(?:json)?\s*|\s*```$', '', text)
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        raise JudgeError(f'JSON が見つからない: {text[:200]}')
    try:
        return json.loads(text[start:end + 1])
    except json.JSONDecodeError as e:
        raise JudgeError(f'JSON の形式エラー: {e}')


def normalize_plan(raw, scene):
    """VLM の出力を検証し、足りない部分を補正した計画を返す。致命的なら JudgeError。"""
    ids = [o.id for o in scene.objects]
    cats = set(scene.categories)
    items = {}
    for it in raw.get('items', []):
        oid = str(it.get('id', '')).strip()
        if oid not in ids:
            continue
        decision = str(it.get('decision', '')).strip().lower()
        if decision not in ('keep', 'discard', 'leave'):
            raise JudgeError(f'{oid} の decision が不正: {decision}')
        category = str(it.get('category', '')).strip().lower()
        if decision == 'discard':
            category = 'trash'
        elif decision == 'leave':
            category = 'stay'
        elif category not in cats:
            category = 'other' if 'other' in cats else sorted(cats)[0]
        items[oid] = {'id': oid, 'decision': decision, 'category': category,
                      'reason': str(it.get('reason', ''))[:60]}
    missing = [i for i in ids if i not in items]
    if missing:
        raise JudgeError(f'判断が抜けている物体: {missing}')

    order = []
    for oid in raw.get('order', []):
        oid = str(oid).strip()
        if oid in items and oid not in order:
            order.append(oid)
    order += [i for i in ids if i not in order]

    # 置き場の数を超えたら、空きのある置き場（other を優先）に回す
    counts = {c: 0 for c in cats}
    spill = ['other'] + [c for c in scene.categories if c != 'other']
    for oid in order:
        it = items[oid]
        c = it['category']
        if c in ('trash', 'stay'):
            continue
        if counts[c] >= len(scene.zones[c].slots):
            c = next((s for s in spill if s in cats and
                      counts[s] < len(scene.zones[s].slots)), None)
            if c is None:
                raise JudgeError('置き場が足りません（scene.yaml の slots を増やしてください）')
            it['category'] = c
        counts[c] += 1

    return {
        'scene_summary': str(raw.get('scene_summary', ''))[:80],
        'items': [items[i] for i in ids],
        'order': order,
        'order_reason': str(raw.get('order_reason', ''))[:80],
    }


def mock_plan(scene):
    items = []
    for o in scene.objects:
        d = o.mock.get('decision', 'keep')
        c = 'trash' if d == 'discard' else o.mock.get('category', 'other')
        items.append({'id': o.id, 'decision': d, 'category': c,
                      'reason': '（mock：scene.yaml の正解）'})
    discard = [o.id for o in scene.objects if o.mock.get('decision') == 'discard']
    keep = [o.id for o in scene.objects if o.id not in discard]
    return {'scene_summary': '（mock モード）', 'items': items,
            'order': discard + keep, 'order_reason': 'ゴミを先に捨てる（mock）'}


def _post(url, headers, body, timeout):
    req = urllib.request.Request(url, data=json.dumps(body).encode('utf-8'),
                                 headers={'Content-Type': 'application/json', **headers})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        raise JudgeError(f'HTTP {e.code}: {e.read().decode("utf-8", "ignore")[:300]}')
    except urllib.error.URLError as e:
        raise JudgeError(f'接続エラー: {e}')


def call_vlm(provider, model, api_key, prompt, jpeg_bytes, timeout=90):
    img = base64.b64encode(jpeg_bytes).decode('ascii') if jpeg_bytes else None
    if provider == 'anthropic':
        content = []
        if img:
            content.append({'type': 'image', 'source': {
                'type': 'base64', 'media_type': 'image/jpeg', 'data': img}})
        content.append({'type': 'text', 'text': prompt})
        res = _post('https://api.anthropic.com/v1/messages',
                    {'x-api-key': api_key, 'anthropic-version': '2023-06-01'},
                    {'model': model, 'max_tokens': 1500,
                     'messages': [{'role': 'user', 'content': content}]}, timeout)
        return ''.join(c.get('text', '') for c in res.get('content', []))
    if provider == 'openai':
        content = [{'type': 'text', 'text': prompt}]
        if img:
            content.append({'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{img}'}})
        res = _post('https://api.openai.com/v1/chat/completions',
                    {'Authorization': f'Bearer {api_key}'},
                    {'model': model, 'response_format': {'type': 'json_object'},
                     'messages': [{'role': 'user', 'content': content}]}, timeout)
        return res['choices'][0]['message']['content']
    if provider == 'gemini':
        parts = [{'text': prompt}]
        if img:
            parts.append({'inline_data': {'mime_type': 'image/jpeg', 'data': img}})
        url = ('https://generativelanguage.googleapis.com/v1beta/models/'
               f'{model}:generateContent?key={api_key}')
        res = _post(url, {}, {'contents': [{'parts': parts}],
                              'generationConfig': {'responseMimeType': 'application/json'}},
                    timeout)
        return ''.join(p.get('text', '') for p in
                       res['candidates'][0]['content']['parts'])
    raise JudgeError(f'未知のプロバイダ: {provider}')


class Judge:

    def __init__(self, provider=None, model=None, log=print):
        self.provider = (provider or os.environ.get('TIDY_VLM_PROVIDER', 'mock')).lower()
        self.model = model or os.environ.get('TIDY_VLM_MODEL') or DEFAULT_MODELS.get(
            self.provider, '')
        self.api_key = os.environ.get(KEY_ENVS.get(self.provider, ''), '')
        self.log = log
        if self.provider != 'mock' and not self.api_key:
            self.log(f'{KEY_ENVS.get(self.provider)} が未設定のため mock で動かします')
            self.provider = 'mock'

    @property
    def label(self):
        return 'mock' if self.provider == 'mock' else f'{self.provider}:{self.model}'

    def decide(self, scene, instruction, jpeg_bytes=None, retries=2):
        """戻り値: (plan, raw_text, elapsed_sec)"""
        if self.provider == 'mock':
            return mock_plan(scene), '', 0.0
        prompt = build_prompt(scene, instruction)
        last = None
        for attempt in range(retries + 1):
            t0 = time.time()
            try:
                text = call_vlm(self.provider, self.model, self.api_key, prompt, jpeg_bytes)
                plan = normalize_plan(extract_json(text), scene)
                return plan, text, time.time() - t0
            except (JudgeError, KeyError, IndexError) as e:
                last = e
                self.log(f'VLM 応答の処理に失敗（{attempt + 1}回目）: {e}')
        self.log(f'VLM が使えなかったため mock の判断で続行します: {last}')
        self.provider = 'mock'      # 以降のラベル・記録を mock にする
        return mock_plan(scene), f'(fallback) {last}', 0.0
