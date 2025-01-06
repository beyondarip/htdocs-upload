import sys
# import os
sys.path.append(r'/home/ris/gps_camera_ws/ML/train/TRANS_1/RT-DETR/rtdetrv2_pytorch')
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from PIL import Image, ImageDraw
from src.core import YAMLConfig
import time  # Tambahkan modul time untuk menghitung FPS

# Pemetaan kelas dari model
class_mapping = {
    0: "AC", 1: "BC", 2: "BL", 3: "BS", 4: "CR", 5: "DP", 6: "EC",
    7: "JC", 8: "LS", 9: "LT", 10: "PA", 11: "PC", 12: "PH", 13: "RT",
    14: "SC", 15: "SL", 16: "SV", 17: "WR"
}

# Fungsi untuk menggambar hasil prediksi pada frame
def draw(im, labels, boxes, scores, thrh=0.6):
    draw = ImageDraw.Draw(im)

    scr = scores
    lab = labels[scr > thrh]
    box = boxes[scr > thrh]
    scrs = scr[scr > thrh]

    for j, b in enumerate(box):
        # Draw bounding box
        draw.rectangle(list(b), outline='red', width=2)
        # Get class name from mapping
        class_id = lab[j].item()
        class_name = class_mapping.get(class_id, "Unknown")
        score = round(scrs[j].item(), 2)
        # Display class name and score
        label = f"{class_name} {score}"
        text_position = (b[0], b[1] - 10)  # Posisi di atas bounding box
        draw.text(text_position, label, fill='blue')

def main(args):
    """main"""
    # Load konfigurasi model
    cfg = YAMLConfig(args.config, resume=args.resume)

    # Load checkpoint model
    if args.resume:
        checkpoint = torch.load(args.resume, map_location='cpu')
        if 'ema' in checkpoint:
            state = checkpoint['ema']['module']
        else:
            state = checkpoint['model']
    else:
        raise AttributeError('Only support resume to load model.state_dict by now.')

    # Load state_dict ke model
    cfg.model.load_state_dict(state)

    # Definisikan model untuk inferensi
    class Model(nn.Module):
        def __init__(self):
            super().__init__()
            self.model = cfg.model.deploy()
            self.postprocessor = cfg.postprocessor.deploy()

        def forward(self, images, orig_target_sizes):
            outputs = self.model(images)
            outputs = self.postprocessor(outputs, orig_target_sizes)
            return outputs

    model = Model().to(args.device)
    model.eval()  # Set model ke mode evaluasi

    # Buka video menggunakan OpenCV
    cap = cv2.VideoCapture(args.im_file)
    if not cap.isOpened():
        raise FileNotFoundError(f"Error: Cannot open video file {args.im_file}")

    # Konfigurasi output video
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output_video.mp4', fourcc, fps, (width, height))

    # Transformasi untuk input ke model
    transforms = T.Compose([
        T.Resize((640, 640)),
        T.ToTensor(),
    ])

    # Variabel untuk menghitung FPS
    frame_count = 0
    start_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Hitung waktu mulai pemrosesan frame
        frame_start = time.time()

        # Convert frame ke PIL Image
        im_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        w, h = im_pil.size
        orig_size = torch.tensor([w, h])[None].to(args.device)

        # Transform frame
        im_data = transforms(im_pil)[None].to(args.device)

        # Inferensi menggunakan model
        with torch.no_grad():
            output = model(im_data, orig_size)
            labels, boxes, scores = output

        # Draw predictions pada frame
        draw(im_pil, labels[0], boxes[0], scores[0])

        # Convert back to numpy array
        annotated_frame = cv2.cvtColor(np.array(im_pil), cv2.COLOR_RGB2BGR)

        # Hitung waktu akhir pemrosesan frame
        frame_end = time.time()
        frame_time = frame_end - frame_start
        fps_current = 1 / frame_time

        # Tambahkan FPS ke frame
        cv2.putText(
            annotated_frame,
            f"FPS: {fps_current:.2f}",
            (10, 30),  # Posisi teks di frame
            cv2.FONT_HERSHEY_SIMPLEX,  # Font
            1,  # Ukuran font
            (0, 255, 0),  # Warna teks (hijau)
            2,  # Ketebalan garis
            cv2.LINE_AA
        )

        # Tulis frame ke output video
        out.write(annotated_frame)

        # Tampilkan hasil (opsional)
        cv2.imshow('Predictions', annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Tambahkan frame count
        frame_count += 1

    # Hitung FPS rata-rata
    end_time = time.time()
    total_time = end_time - start_time
    avg_fps = frame_count / total_time
    print(f"Average FPS: {avg_fps:.2f}")

    # Release resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, required=True, help='Path to config file')
    parser.add_argument('-r', '--resume', type=str, required=True, help='Path to model checkpoint')
    parser.add_argument('-f', '--im-file', type=str, required=True, help='Path to input video')
    parser.add_argument('-d', '--device', type=str, default='cpu', help='Device to run inference on')
    args = parser.parse_args()
    main(args)
