import cv2
from PIL import Image, ImageDraw

# Generate ArUco marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
marker_id = 12
marker_size_px = 200
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

# Convert to PIL Image
pil_marker = Image.fromarray(marker_image)

# Define physical size in mm
physical_size_mm = 30
dpi = 300  # Standard print DPI
pixels_for_size = int((physical_size_mm / 25.4) * dpi)

# Resize marker
pil_marker = pil_marker.resize(
    (pixels_for_size, pixels_for_size), Image.Resampling.LANCZOS
)

# Create A4 page (210mm x 297mm)
a4_width_px = int((210 / 25.4) * dpi)
a4_height_px = int((297 / 25.4) * dpi)
a4_page = Image.new("RGB", (a4_width_px, a4_height_px), "white")

# 4 corners layout with equal symmetric spacing
# A4 dimensions in mm
a4_width_mm = 210
a4_height_mm = 297
marker_size_mm = physical_size_mm

# Define equal margin from edges in mm (symmetric)
margin_mm = 20  # 20mm margin from each edge

# Convert margin to pixels
margin_px = int((margin_mm / 25.4) * dpi)

# Place 4 markers in corners
# Top-left
a4_page.paste(pil_marker, (margin_px, margin_px))
# Top-right
a4_page.paste(
    pil_marker,
    (a4_width_px - margin_px - pixels_for_size, margin_px),
)
# Bottom-left
a4_page.paste(
    pil_marker,
    (margin_px, a4_height_px - margin_px - pixels_for_size),
)
# Bottom-right
a4_page.paste(
    pil_marker,
    (
        a4_width_px - margin_px - pixels_for_size,
        a4_height_px - margin_px - pixels_for_size,
    ),
)

# Save as PDF
a4_page.save("aruco_marker.pdf", dpi=(dpi, dpi))
print(
    f"Saved: 4x {physical_size_mm}mm x {physical_size_mm}mm markers (ID {marker_id}) in 2x2 grid on A4 at {dpi} DPI"
)
