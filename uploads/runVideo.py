import cv2
import time
from ultralytics import YOLO
import torch

# Pastikan CUDA tersedia
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

# Load model
model = YOLO('runs/detect/train5/weights/best.pt')  # Ganti dengan path ke model YOLO Anda
model.to(device)  # Pindahkan model ke GPU

# Load video
video_path = 'testvid/yqq.mp4'  # Path ke video input
output_path = 'testvid/output_video_jetson.mp4'  # Path untuk menyimpan video output

cap = cv2.VideoCapture(video_path)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, cap.get(cv2.CAP_PROP_FPS),
                      (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

frame_count = 0
total_time = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Mulai penghitungan waktu
    start_time = time.time()

    # Perform prediction (gunakan GPU jika tersedia)
    results = model.predict(source=frame, device=device, save=False, verbose=False)

    # Hitung waktu untuk frame ini
    elapsed_time = time.time() - start_time
    total_time += elapsed_time
    frame_count += 1

    # Hitung FPS saat ini
    fps = frame_count / total_time if total_time > 0 else 0

    # Visualize results on the frame
    annotated_frame = results[0].plot()  # Menggambar bounding box pada frame

    # Tambahkan teks FPS ke frame
    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Write the frame into the output file
    out.write(annotated_frame)

    # Optionally display the frame (uncomment to show live preview)
    # cv2.imshow('YOLO Prediction', annotated_frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

cap.release()
out.release()
cv2.destroyAllWindows()

print(f"FPS rata-rata selama prediksi: {fps:.2f}")
print(f"Video hasil prediksi disimpan di: {output_path}")
