"""Regenerate the Spiral Bevel command icons from a render of a real pair.

    py -3.10 tools/make_icons.py        # needs Pillow + numpy
    py -3.10 tools/sb.py render '{"out": ".../resources/src/icon_source.png", ...}'
                                        # to re-render the source in Fusion

Source: resources/src/icon_source.png - a 12 : 18 pair, 35 deg spiral, face 0.59 Re
(so the spiral teeth, not the flat toes, carry the silhouette),
blue anodised gear and polished aluminium pinion, rendered by the add-in's own
`render` dev job on a transparent background (so the icon is the real geometry).

This is Horological Gears' icon pipeline (resources/make_icons.py there),
unchanged except for the paths, so the two add-ins' icons read as a family.
Its reasoning, kept below, applies here as is.

    WHICH FILES FUSION ACTUALLY WANTS
    Shipping only 16x16/32x32/64x64 is NOT enough: the small icon in the
    Create dropdown picks up, but the large promoted toolbar button stays
    on its old/default bitmap. Fusion's own command resources (e.g.
    .../InternalAddins/ElectronicsPackageGenerator/Resources/) ship a full
    matrix, and on a HiDPI display -- this machine is 3840x2160 -- Fusion
    asks for the "@2x" file for every logical size:

        16x16.png   16px     16x16@2x.png    32px    <- menu / list rows
        32x32.png   32px     32x32@2x.png    64px    <- LARGE toolbar button
        64x64.png   64px     64x64@2x.png   128px    <- large @ HiDPI

    plus "-dark" copies for Fusion's dark theme and "-disabled" copies for
    the greyed-out state. All of them are written here.

    Cross-checked against Fusion's own Create-panel commands (Extrude,
    Sweep, Rib, Loft, Mirror under
    .../Applications/Fusion/Fusion/UI/FusionUI/Resources/): most ship only
    16x16 + 16x16@2x + 32x32 + 32x32@2x (plus -dark), i.e. the promoted
    LARGE button is served by 32x32@2x at 64 px. 64x64 is written here too
    (Mirror ships it) but is not what the large button uses.

    NOTE: Fusion loads these lazily and caches them for the life of the
    process. RESTART FUSION to pick up new artwork. Stop+Run from the
    Add-Ins dialog does NOT work, and neither does pointing the command at
    a brand-new resource folder path -- both were tried and measured, and
    Fusion never reopened the files.

FRAMING
    The subject is cropped tight to its alpha bounding box and centred in
    a square, reserving only the 1 px the contour needs. Cropping tighter
    to also fill the height was tried and rejected: the pinion sits at the
    lower right, so any square narrower than the subject slices it off the
    corner. Whatever vertical air is left is the cost of a wide subject in
    a square icon.

CONTOUR
    A 1 px contour in a DARKER SHADE OF THE ARTWORK'S OWN EDGE COLOUR,
    added at the final resolution. Without it the pale pinion dissolves
    into Fusion's light panel. See add_outline for why it is neither flat
    black nor 2 px.

DOWNSAMPLING
    Done on PREMULTIPLIED alpha. The render's fully transparent pixels
    carry a dark grey RGB; a straight RGBA resize bleeds that into the
    edges and dirties every icon -- very obvious at 16 px.

SHARPENING
    Graduated by REAL pixel size (not by logical name), since the small
    icons lose the most detail. Kept mild: heavy unsharp leaves a light
    halo along the rim.
"""
import os
import sys

from PIL import Image, ImageFilter
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ICON_DIR = os.path.join(ROOT, "resources", "SpiralBevel")
SOURCE = os.path.join(ROOT, "resources", "src", "icon_source.png")
# (source is chosen by modification time; see find_source)


def find_source():
    if not os.path.exists(SOURCE):
        sys.exit("no source render at %s" % SOURCE)
    return SOURCE


# The outline reserves its own border, so the subject needs no extra margin.
MARGIN = 0.0

# logical name -> (1x pixels, 2x pixels)
VARIANTS = {
    "16x16": (16, 32),
    "32x32": (32, 64),
    "64x64": (64, 128),
}

# real pixel size -> unsharp percentage
SHARPEN = {16: 70, 32: 50, 64: 25, 128: 15}

# A hairline dark contour, so the pale pinion does not dissolve into Fusion's
# light panel.
#
# ONE device pixel at every size, applied at the final resolution. Scaling it
# with the pixel density (2 px on the @2x files, for physically equal weight)
# was tried and rejected: this artwork is finely toothed, and at 2 px the
# contour starts closing the gaps between teeth on the 32 px menu icon. 1 px
# keeps the teeth separated and still lifts the whole silhouette off a light
# background.
OUTLINE_PX = 1

# The contour is NOT flat black. Fusion's own icons outline each shape in a
# darker shade of that shape's own colour -- dark blue around blue, dark grey
# around grey -- which reads as contrast and as volume rather than as a
# cartoon sticker outline. So the edge colour is sampled from the artwork and
# multiplied down by this factor.
OUTLINE_DARKEN = 0.42
# ...but never let a very pale edge (the near-white pinion) produce a contour
# too light to separate from a white panel.
OUTLINE_MAX_LUMA = 150.0


def premultiplied_resize(img, size):
    a = np.asarray(img).astype(np.float64)
    rgb, alpha = a[..., :3], a[..., 3:4] / 255.0
    pm = np.concatenate([rgb * alpha, alpha * 255.0], axis=-1)
    small = Image.fromarray(np.clip(pm, 0, 255).astype(np.uint8), "RGBA") \
        .resize((size, size), Image.LANCZOS)
    b = np.asarray(small).astype(np.float64)
    prgb, palpha = b[..., :3], b[..., 3:4] / 255.0
    with np.errstate(divide="ignore", invalid="ignore"):
        straight = np.where(palpha > 0, prgb / np.maximum(palpha, 1e-6), 0.0)
    return Image.fromarray(np.concatenate(
        [np.clip(straight, 0, 255), palpha * 255.0], axis=-1).astype(np.uint8), "RGBA")


def sharpen(img, percent):
    if percent <= 0:
        return img
    rgb = img.convert("RGB").filter(
        ImageFilter.UnsharpMask(radius=0.6, percent=percent, threshold=0))
    out = rgb.convert("RGBA")
    out.putalpha(img.getchannel("A"))
    return out


def add_outline(icon, width_px):
    """Lay a contour under the icon, hugging its silhouette.

    Shape:  the alpha channel is dilated by `width_px` -- each opaque pixel
            grows into its neighbours, a MaxFilter over a 2n+1 window -- and
            the original icon is composited back on top, so the contour only
            ever appears OUTSIDE the artwork. Nothing is overpainted.

    Colour: sampled from the artwork's own edge rather than a flat black.
            The premultiplied colour is bled outward with a box blur, then
            darkened, giving dark blue around the blue wheel and dark grey
            around the pale pinion -- Fusion's own icon idiom, which reads as
            volume instead of as a sticker outline.

    Done at the final resolution on purpose: dilating the big source and then
    downsampling would blur the contour away at 16 px, where it matters most.
    """
    if width_px < 1:
        return icon

    a = np.asarray(icon).astype(np.float64)
    rgb, alpha = a[..., :3], a[..., 3:4] / 255.0

    # Bleed the edge colour outward on PREMULTIPLIED values, so fully
    # transparent pixels contribute no colour of their own.
    pm = Image.fromarray(
        np.clip(rgb * alpha, 0, 255).astype(np.uint8), "RGB"
    ).filter(ImageFilter.BoxBlur(width_px))
    aw = Image.fromarray(
        np.clip(alpha[..., 0] * 255.0, 0, 255).astype(np.uint8), "L"
    ).filter(ImageFilter.BoxBlur(width_px))

    pm_a = np.asarray(pm).astype(np.float64)
    aw_a = np.asarray(aw).astype(np.float64)[..., None] / 255.0
    with np.errstate(divide="ignore", invalid="ignore"):
        edge = np.where(aw_a > 1e-6, pm_a / np.maximum(aw_a, 1e-6), 0.0)

    edge *= OUTLINE_DARKEN
    # Cap the brightness so a near-white edge still separates from a white panel.
    luma = (0.2126 * edge[..., 0] + 0.7152 * edge[..., 1]
            + 0.0722 * edge[..., 2])[..., None]
    scale = np.where(luma > OUTLINE_MAX_LUMA,
                     OUTLINE_MAX_LUMA / np.maximum(luma, 1e-6), 1.0)
    edge = np.clip(edge * scale, 0, 255)

    grown = np.asarray(
        icon.getchannel("A").filter(ImageFilter.MaxFilter(2 * width_px + 1))
    ).astype(np.float64)[..., None]

    contour = Image.fromarray(
        np.concatenate([edge, grown], axis=-1).astype(np.uint8), "RGBA")
    contour.alpha_composite(icon)
    return contour


def disabled(img):
    """Greyed-out state: desaturate and drop to 40% opacity."""
    a = np.asarray(img).astype(np.float64)
    rgb, alpha = a[..., :3], a[..., 3:4]
    lum = (0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2])
    grey = np.repeat(lum[..., None], 3, axis=-1)
    # pull toward mid grey so it reads as inactive on either theme
    grey = grey * 0.55 + 128 * 0.45
    out = np.concatenate([np.clip(grey, 0, 255), alpha * 0.40], axis=-1)
    return Image.fromarray(out.astype(np.uint8), "RGBA")


def main():
    src = find_source()
    im = Image.open(src).convert("RGBA")
    bbox = im.getchannel("A").getbbox()
    if bbox is None:
        sys.exit("source render is fully transparent")

    subject = im.crop(bbox)
    w, h = subject.size
    side = int(round(max(w, h) * (1.0 + 2 * MARGIN)))
    # Centre on the subject's bounding box, so the margins are equal on
    # opposing sides. (Centring on the opaque centroid instead pulls toward
    # the big wheel and pushes the pinion into the corner.)
    ox, oy = (side - w) // 2, (side - h) // 2
    square = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    square.paste(subject, (ox, oy))

    print("source   %s" % os.path.basename(src))
    print("         %dx%d" % im.size)
    print("subject  %s -> %dx%d (%.2f:1)" % (bbox, w, h, w / float(h)))
    print("square   %dx%d -- subject fills %.0f%% width, %.0f%% height"
          % (side, side, 100.0 * w / side, 100.0 * h / side))
    print("centred  margins L/R = %d/%d px, T/B = %d/%d px"
          % (ox, side - w - ox, oy, side - h - oy))
    print()

    def render(px, scale):
        """One finished icon: subject inset by the contour width, then the
        contour grown into the reserved border."""
        n = OUTLINE_PX
        inner = px - 2 * n
        art = sharpen(premultiplied_resize(square, inner), SHARPEN.get(px, 0))
        canvas = Image.new("RGBA", (px, px), (0, 0, 0, 0))
        canvas.paste(art, (n, n))
        return add_outline(canvas, n)

    # Cache on (pixels, scale): 32 px is produced BOTH as 16x16@2x (scale 2,
    # 2 px contour) and as 32x32 (scale 1, 1 px contour) -- same pixel size,
    # deliberately different contour weight.
    cache = {}
    for name, (px1, px2) in sorted(VARIANTS.items()):
        for scale, px in ((1, px1), (2, px2)):
            if px not in cache:
                cache[px] = render(px, scale)
                print("  rendered %3dpx @%dx  subject %3dpx + %dpx contour"
                      % (px, scale, px - 2 * OUTLINE_PX, OUTLINE_PX))
    print()

    os.makedirs(ICON_DIR, exist_ok=True)
    written = 0
    for name, (px1, px2) in sorted(VARIANTS.items()):
        for suffix, px, scale in (("", px1, 1), ("@2x", px2, 2)):
            base = cache[px]
            # normal + dark share the artwork: the render reads on both themes
            for theme in ("", "-dark"):
                path = os.path.join(ICON_DIR, "%s%s%s.png" % (name, theme, suffix))
                base.save(path, "PNG", optimize=True)
                written += 1
            path = os.path.join(ICON_DIR, "%s-disabled%s.png" % (name, suffix))
            disabled(base).save(path, "PNG", optimize=True)
            written += 1
        print("  %-8s %3dpx (1x) / %3dpx (@2x)  + dark + disabled" % (name, px1, px2))

    print()
    print("wrote %d files to %s" % (written, ICON_DIR))
    print("RESTART FUSION to pick these up -- Stop+Run does not reload bitmaps.")


if __name__ == "__main__":
    main()
