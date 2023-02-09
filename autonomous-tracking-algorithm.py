# Değişik kaynaklardan bakarak bir otonom izleme algoritması yazdım çalıştırabilirsek mükemmel olur #

import cv2
import numpy as np

# İzleme algoritmasını tanımlayın
def track_uav(frame, roi):
    # Çerçeveyi gri tonlamaya dönüştür
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # IHA için HSV renk aralığını tanımlayın
    lower_hsv = np.array([0, 0, 0])
    upper_hsv = np.array([180, 255, 80])
    
    # IHA'yı çıkarmak için çerçeveyi eşikleyin
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # IHA'yı izlemek için CAMShift algoritmasını kullanın
    track_window = cv2.CamShift(mask, roi, term_crit)
    center, roi = cv2.meanShift(mask, roi, term_crit)
    
    # İHA'nın etrafına bir dikdörtgen çizin
    x, y, w, h = track_window
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    # Güncellenen çerçeveyi döndür
    return frame

# Videoyu yükle
video = cv2.VideoCapture('uav_video.avi')

# CAMShift algoritması için sonlandırma kriterlerini tanımlayın
term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

# İHA için ilk ROI'yi seçin
_, frame = video.read()
roi = cv2.selectROI(frame, False)

# Videonun her karesini işleyin
while True:
    # Sonraki kareyi oku
    _, frame = video.read()
    
    # Başka çerçeve yoksa döngüyü kırın
    if frame is None:
        break
    
    # İHA'yı geçerli çerçevede izleyin
    frame = track_uav(frame, roi)
    
    # Güncellenen çerçeveyi göster
    cv2.imshow('UAV Tracking', frame)
    
    # 'q' tuşuna basılırsa döngüyü kırın
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Videoyu yayınlayın ve tüm pencereleri yok edin
video.release()
cv2.destroyAllWindows()
