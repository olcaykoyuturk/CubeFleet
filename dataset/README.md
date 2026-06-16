# Cube Dataset — Raw Captures

ESP32-CAM HVGA (480×320) ile [pc/dataset_capture.py](../pc/dataset_capture.py)
kullanilarak toplanan ham görseller. Henuz etiketlenmemis.

## Etiketli versiyon

Roboflow projesinde etiketlendi:
- **Workspace:** olcays
- **Project:** cube-ajbii
- **Version:** 1 (YOLOv11 format)
- **URL:** https://universe.roboflow.com/olcays/cube-ajbii/dataset/1
- **Class:** Cube (tek sinif)
- **Split:** 222 train / 24 valid
- **Augmentation:** Roboflow default (brightness, rotation, hue)

## Egitim sonucu

`models/cube_best.pt` — YOLO11s @ 480x480, 50 epoch (T4 GPU, ~12 dk):

| Metrik | Sonuc |
|--------|-------|
| mAP@50 | 0.983 |
| mAP@50-95 | 0.913 |
| Precision | 0.968 |
| Recall | 0.997 |

Egitim akisi: [docs/yolo-train.md](../docs/yolo-train.md)

## Yeni capture toplama

```powershell
& "c:\Projelerim\AGV\.venv\Scripts\python.exe" `
    c:\Projelerim\AGV\pc\dataset_capture.py
```

Default cikti: `dataset/cube_YYYYMMDD_HHMMSS_NNN.jpg`
