#!/usr/bin/env python
import sys
from PIL import Image

def main():
    if len(sys.argv) < 2:
        print("Usage: python format_figure.py <input_image>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    try:
        # Open the input image (PPM file)
        img = Image.open(input_file)
    except Exception as e:
        print(f"Error opening image {input_file}: {e}")
        sys.exit(1)
    
    # Flip the image vertically (since glReadPixels returns bottom-up)
    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    
    # Define target size (3.5" x 2.5" at 300 DPI)
    target_dpi = 300
    target_width_inch = 3.5
    target_height_inch = 2.5
    target_width_px = int(target_width_inch * target_dpi)   # 1050 pixels
    target_height_px = int(target_height_inch * target_dpi)  # 750 pixels
    
    # Determine the simulation image's size
    sim_width, sim_height = img.size
    # Calculate scale factor to fit the simulation image into the target dimensions while preserving aspect ratio.
    scale_factor = min(target_width_px / sim_width, target_height_px / sim_height)
    new_width = int(sim_width * scale_factor)
    new_height = int(sim_height * scale_factor)
    
    # Resize the simulation image with high-quality resampling.
    img_resized = img.resize((new_width, new_height), resample=Image.Resampling.LANCZOS)
    
    # Create a new blank (white) image with the target dimensions.
    final_img = Image.new("RGB", (target_width_px, target_height_px), (255, 255, 255))
    
    # Center the resized simulation image on the white background.
    x_offset = (target_width_px - new_width) // 2
    y_offset = (target_height_px - new_height) // 2
    final_img.paste(img_resized, (x_offset, y_offset))
    
    # Save the final image as a PNG with the specified DPI.
    output_file = "formatted_figure.png"
    try:
        final_img.save(output_file, dpi=(target_dpi, target_dpi))
        print(f"Formatted figure saved as {output_file}")
    except Exception as e:
        print(f"Error saving formatted image: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
