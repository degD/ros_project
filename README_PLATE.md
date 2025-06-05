# PLAKA OKUMA

Simülasyon ortamı araç plakaları gerçek hayatta olduğundan daha nettir. Arka plan mat beyaz, yazılar mat siyah. Tabii farklı açılardan araç fotoğrafları çekileceğinden farklı açılardan plaka tespiti yapabilen bir sistem gerekmektedir. Araçların fotoğrafları çekilir. Ardından bu fotoğraflarda plaka tespiti yapılmaya çalışılır. 

- Şimdilik araçların merkez noktasına göre ön, arka, sağ, soldan fotoğrafları çekilir ve plaka okuma yapılmaya çalışılır.
- Gelecekte araç ile aynı doğrultuya geçilip yalnız ön ve arkaya bakılarak da çalıştırılabilir.

## KURULUM

Plaka tespiti için [Plate Recognizer](https://platerecognizer.com/) servisi kullanılmakta. Plakalar API call yapılarak tespit edilmekte. Hesap başı ayda 2500 tespite kadar ücretsiz. Daha sonra local bir kütüphaneye geçilebilir.

- Testlerde iyi sonuç veriyor.
- Ancak font kaynaklı sıfırı (0) okumakta zorlanıyor (ortasında nokta var).
- Duruma göre hem kütüphane hem de modeller güncellenebilir.