# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Approved minimal OmniLink video overlay. Source pixels stay native."""
from PIL import Image, ImageDraw, ImageFont


def overlay(frame, prompt=None, answer=None, robot='HUSKY',
            font_path='C:/Windows/Fonts/segoeui.ttf'):
    """Return an RGB frame with one prompt or answer; scale from 1600x900."""
    scale=frame.width/1600
    px=lambda n: round(n*scale)
    font=lambda n: ImageFont.truetype(font_path,px(n))
    layer=Image.new('RGBA',frame.size,(0,0,0,0))
    draw=ImageDraw.Draw(layer)
    draw.text((px(36),px(28)),'OmniLink',font=font(24),
              fill=(245,247,249,235),stroke_width=px(1),stroke_fill=(0,0,0,30))
    text=answer if answer is not None else prompt
    if text:
        f=font(30 if answer is not None else 38)
        lines=[]
        for paragraph in text.split('\n'):
            line=''
            for word in paragraph.split():
                candidate=(line+' '+word).strip()
                if line and draw.textlength(candidate,font=f)>px(1370):
                    lines.append(line);line=word
                else:line=candidate
            if line:lines.append(line)
        if len(lines)>2:
            raise ValueError('Keep an OmniLink prompt or answer to two lines.')
        width=round(max(draw.textlength(line,font=f) for line in lines))+px(64)
        line_height=px(48 if answer is None else 40)
        label_height=px(23) if answer is not None else 0
        height=line_height*len(lines)+px(38)+label_height
        left=(frame.width-width)//2; top=frame.height-px(50)-height
        draw.rounded_rectangle((left,top,left+width,top+height),radius=px(18),fill=(12,20,29,235))
        y=top+px(15)
        if answer is not None:
            draw.text((left+px(32),y-px(4)),robot,font=font(16),fill=(111,231,208,255))
            y+=label_height
        for line in lines:
            draw.text((left+px(32),y),line,font=f,fill=(246,248,250,255));y+=line_height
    return Image.alpha_composite(frame.convert('RGBA'),layer).convert('RGB')
