import cv2
import torch
from torchvision import transforms
from PIL import Image
from model import FasterRCNN
import time

# Muat model_config dari parameter yang telah ditentukan sebelumnya
model_config = {
    "im_channels": 3,
    "aspect_ratios": [0.5, 1, 2],
    "scales": [128, 256, 512],
    "min_im_size": 600,
    "max_im_size": 1000,
    "backbone_out_channels": 512,
    "fc_inner_dim": 1024,
    "rpn_bg_threshold": 0.3,
    "rpn_fg_threshold": 0.7,
    "rpn_nms_threshold": 0.7,
    "rpn_train_prenms_topk": 12000,
    "rpn_test_prenms_topk": 6000,
    "rpn_train_topk": 2000,
    "rpn_test_topk": 300,
    "rpn_batch_size": 256,
    "rpn_pos_fraction": 0.5,
    "roi_iou_threshold": 0.5,
    "roi_low_bg_iou": 0.0,
    "roi_pool_size": 7,
    "roi_nms_threshold": 0.3,
    "roi_topk_detections": 100,
    "roi_score_threshold": 0.05,
    "roi_batch_size": 128,
    "roi_pos_fraction": 0.25,
}

# Muat model
model = FasterRCNN(model_config, num_classes=19)  # Sesuaikan jumlah kelas
model.load_state_dict(torch.load(r"/home/ris/gps_camera_ws/ML/train/FASTER_RCNN_F/FasterRCNN-PyTorch/model latih/faster_rcnn_road.pth",map_location=torch.device("cpu")))
model.eval()

# Pastikan model berada di perangkat yang sesuai (GPU atau CPU)
#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device( "cpu")
model = model.to(device)

# Daftar nama kelas
class_names = ["background", 'AC', 'BC', 'EC', 'JC', 'LT', 'SC', 'BS', 'CR', 'DP', 'LS', 'PH', 'RT', 'SV', 'SL', 'PA', 'BL', 'WR', 'PC']

# Transformasi untuk preprocessing gambar (resize, normalize)
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Fungsi untuk memproses frame dan melakukan prediksi
def process_frame(frame):
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    input_image = transform(image).unsqueeze(0).to(device)  # Tambahkan batch dimension

    with torch.no_grad():
        _, frcnn_output = model(input_image)

    boxes = frcnn_output['boxes'].cpu().numpy()
    scores = frcnn_output['scores'].cpu().numpy()
    labels = frcnn_output['labels'].cpu().numpy()

    return boxes, scores, labels

# Fungsi untuk menggambar deteksi pada frame
def draw_detections(frame, boxes, scores, labels, score_threshold=0.5):
    for box, score, label in zip(boxes, scores, labels):
        if score < score_threshold:
            continue

        x1, y1, x2, y2 = box.astype(int)
        class_name = class_names[label] if label < len(class_names) else f"class{label}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{class_name}: {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Buka video
video_path = r"/home/ris/gps_camera_ws/ML/train/FASTER_RCNN_F/FasterRCNN-PyTorch/1223.mp4"
cap = cv2.VideoCapture(video_path)

# Tentukan format output video
output_path = r"/home/ris/gps_camera_ws/ML/train/FASTER_RCNN_F/FasterRCNN-PyTorch/outputh32.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))
out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# Variabel untuk menghitung FPS rata-rata
total_time = 0
frame_count = 0

# Proses setiap frame
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Waktu mulai untuk frame ini
    frame_start_time = time.time()

    # Lakukan prediksi
    boxes, scores, labels = process_frame(frame)

    # Gambar hasil deteksi pada frame
    draw_detections(frame, boxes, scores, labels)

    # Hitung waktu proses untuk frame ini
    frame_end_time = time.time()
    frame_time = frame_end_time - frame_start_time
    total_time += frame_time

    # Hitung FPS untuk frame ini
    fps = 1 / frame_time if frame_time > 0 else 0
    frame_count += 1
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    # Tampilkan hasil di layar (opsional)
    cv2.imshow('Video', frame)

    # Simpan frame ke file output
    out.write(frame)

    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Hitung rata-rata FPS setelah semua frame diproses
average_fps = frame_count / total_time if total_time > 0 else 0

# Tampilkan hasil rata-rata FPS
print(f"Total Frames Processed: {frame_count}")
print(f"Total Time: {total_time:.2f} seconds")
print(f"Average FPS: {average_fps:.2f}")

# Bersihkan resource
cap.release()
out.release()
cv2.destroyAllWindows()
