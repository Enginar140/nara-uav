"""Gorus-tabanli takip/kilitlenme alt sistemi (Savasan IHA gorevi).

Katmanlar: detection (tipler+geometri), lock_evaluator (sartname kilit durum
makinesi), target_estimator (Kalman tahmin), centering_pid (merkeze getirme).
Agir/optional moduller (YOLO, ROI, video) ayri dosyalarda, cv2/yolo yoksa
kirilmaz.
"""
