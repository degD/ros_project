# ros_project

## Senaryo:
Kullanıcı harita üzerinden bir alan seçecek. Drone bu alan içinde arabaları tespit edip plakalarını okuyacak.
Hangi plakaların bulunduğunu belirleyecek. Kullanıcının seçtiği alan "arama alanı" olsun, ama her yeri seçemez
sonuçta. Droneun hizmet verdiği alan "hizmet alanı" diye adlandırılsın. Basit objeden kaçınma da var. 

Basit bir WebUI olsun. Drone konumu gösterilsin ve hizmet alanı gösterilsin. Bir OSM widgeti olabilir. "Arama alanı"
içinde hareket aralıklar ile waypoint konulması ile sağlanabilir. Bir "alan belirle" tuşu ve bir "onayla" tuşu olsun.
Onaylayınca waypointler de harita üzerinde gösterilsin. Drone kontrolü için de iki tuş olabilir. Bir "başlat" ve bir
"duraklat" tuşu. Hangi periyotlarla "arama alanında" gezileceğini girebileceğimiz bir "input" olsun. Kalkış noktasında
olsun drone. "Başlat" ile alanda gezinsin ve geri kalkış noktasına dönsün. Periyot bitimine kadar beklesin ve yeni
periyot ile tekrar hareket etsin. Her periyotta plakalar ve araç konumları listelensin. Plakaların raporlanması, 
başlangıç için bir .txt dosyası aracılığı ile olabilir. Her periyot için bir dosya oluşturulur. "Hizmet alanı" şimdilik
bir config dosyası ile belirlenen koordinat ve yarıçap ile gösterilsin. Tespit edilen araçlar da harita üzerinde işaretlenebilir.
Belki plaka okuma başarı durumuna göre "yeşil veya kırmızı" işaretleme yapılabilir.

WebUI basit bir şekilde Flask ile gerçekleştirilebilir. Server, duruma göre ROS ile iletişime geçer. Bir de her şeyi 
çalıştıracak bir .launch dosyası lazım. ROS için Ardupilot veya PX4 ile Mavros kullanımı büyük kolaylık sağlar. SITL
kurulum yapılmalı. Droneun gazebodan görüntüyü alması ve objeden kaçınmayı gazeboda yapması önemli. Ardupilot ile 
normalde harita verisine göre bina yüksekliklerine dikkat ediliyordu. Bunu nasıl dahil edebilirim bakmalıyım. Özellikle
üç temel nokta kalıyor: 
1. Gezinme sırasında tüm alanın görüntüsünü aldığımdan emin olacağım bir algoritma
2. Gazebo haritası, araba modelleri, plakalar, tabii kampüs gibi bir alanın gazeboda yüklenmesi için yöntem
3. Görüntüleri almak, işlemek için YOLO modeli

Bunların yanı sıra tespit edilen bir otomobil için plakanın nasıl okunacağı da ayrı bir sorun. Plakası ön veya arkada olabilir.
Arabanın yaklaşık tepesinde konumlanırsa drone, araba çevresinde plaka görünceye kadar küçük bir tur atabilir. Eğer tur sonunda 
plaka bulunmazsa arabayı "olası hatalı tespit" olarak işaretler ve gezme işine kaldığı yerden devam eder. Aynı plakayı yeniden
görürse onu yine de listeye ekler. Sonuçta amaç hangi plakaların olduğunu listelemek. Görüntü işlemek ve plaka konusunda model
bulunması veya eğitilmesi gerekebilir. Tüm bu işlem sırasında tercihen drone engellerden hala kaçınmalı.

Hareketli araçlar için daha farklı bir yöntem uygulamalıyım. Araç hareket ettiği için aslında droneun onun 
çevresinde tur atmasına gerek yok. Çünkü araç uzaklaştıkça plakası görünebilir bir açıya gelecek. Yine de hareketli
araçların plakasının okunması bu proje kapsamında tam bulunmuyor. Amaç daha çok park halinde olan arabaların plakalarını okumak.
Burada iki seçenek mevcut:
1. Aracın hareket edip etmediğini algılayıp ona göre aksiyon al
2. Ya da her durumda araba sabitmiş gibi davran
Eğer aracın hareket edip etmediği tespit edilmek isteniyorsa droneun da hareketli olduğu göz önünde bulundurulmalı. Bu konuda en temel
yöntem droneu sabitleyip aracın hareketli olup olmadığını anlamak olur. Ama bunu şimdilik proje kapsamından çıkarmak ve
araçların hepsini sabit kabul etmek daha makul olacak gibi.

Önce "arama alanı" droneun yüksekliğine göre eş karelere ayrılır ve drone bu karelerin merkezlerini ziyaret eder. Mesela yerden 20m
yüksekte olur. Kamera aşağı dönük. Bir araba tespit edince araca doğru alçalır. Bu sırada plakasını gördüyse kalkar, devam eder. Peki ya çokça araba
yan yanaysa? Bir park yeri gibi. Araç sayısı ile özgün plaka sayısının aynı olması lazımdır. Drone ile plaka tespit için nasıl bir yöntem uygulanacağı
araştırılmalı.

(arabalar: https://sketchfab.com/3d-models/generic-passenger-car-pack-20f9af9b8a404d5cb022ac6fe87f21f5)

## Geliştirme Planı:
1. WebUI için demo hazırla
2. WebUI ile droneun iletişim kurması, kontrolü, boş gazebo dünyasında gezinmesi
3. Gazebo dünyasına arabalar eklenmesi ve araba tespiti, WebUI üzerinde işaretleme
4. Gazebo için plakalar ve plaka okuma algoritması
5. Gazebo dünyasının doldurulması
6. Basit objeden kaçınma (Veya PX4/Ardupilot ile uygula)

## Harita
Özellikle haritanın oluşturulması ve gazeboya yüklenmesi sırasında sıkıntı ile karşılaşıldı. Gazebo dünyasında model yüklenemiyor.
İki seçenek var:
1. Modeli yüklemeye çalışmaya devam et.
2. Gazebo içinden bir mini dünya oluştur.

## İlerleme:
- Basit bir demo hazırlandı. Aşağıdaki sıra ile komutlar çağırılarak demo çalıştırılabilir.
- Temel akış (eksiklikler olsa da) tamam gibi.
- Basit WBM var.

## Eksikler:
- WBM ve demo bağımsız.
- Demo dünya ve araçlar çok basit.
- Araç textureları bozuk.
- Drone çok yavaş ve hareket hatalı.
- Engelden kaçınma mevcut değil.

## Demo çalıştırma:
İlk olarak:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5
chmod +x place_sim_files.sh
./place_sim_files.sh
```

Ayrı bir terminalde:
```
roslaunch rotors_gazebo rotors_hovering_vi_extra_camera.launch mav_name:=firefly world_name:=parked_cars_demo
```

Son olarak yine proje içinde:
```
python ros_library.py
```

## KALAN İŞLER

### 1. Lokal engelden kaçınma ve rota algoritması
- Octomap veya PX4 kullanılabilir. Hedef nokta için engeller hesaba katılarak rota çizer
- Engelleri anlamak için droneun hareket ettiği yöne doğru bakması lazım
- Bu da tüm hareket sisteminin yeniden yazılması anlamına geliyor
- Belki mavros ve entegre engelden kaçınması da kullanılabilir
- Denenen yöntem: depthpoint + octomap ile occupancy grid, occupancy grid'den de costmap
- Ama costmap çalışmıyor

### 2. WebUI iyileştirmeleri
- ✅ Harita üzerinde drone, tespit edilen araçlar, merkez, plakalar vs. gözüksün
- ✅ CONFIG dosyasını library vs. de okusun
- ✅ Alan çizmede servis alanı sınırı
- ✅ Kareleri ziyaret etmeyi harita olmasa da yap
- Periot mantığını kontrol et, start ve stop.
- Arayüz iyileştirmesi

### 3. Harita geliştirmeleri
- Artık WebUI daha iyi. Buna katkı için harita da geliştirilebilir
- Gazebo performansı nasıl iyileştirilir?
- Her kare yaklaşık 92m2, 4 kare istesek bu alan gazebo'da nasıl çalışır?
- Gezinme sonunda drone servis merkezi yani ev konumuna dönsün

### 4. Drone hareket iyileştirmeleri
- Eğer engelden kaçınma bu işin çözümü için yeterli olmadıysa dronun sallanmasını engelle
- Hızlı hareket etmesini sağla

### 5. Rapor güncelleme
- Raporda eksikler ve yanlışlar olabilir. Baştan sona gözden geçir.
- Raporu tekrar oluştur.
- Makale için de geçerli.

### 6. Araç tespit iyileştirmeleri
- mmrotate özellikle siyah arabaları algılamıyor. Buna çözüm üret.
- Modeller ve yöntemler dene.
- Plaka okumayı geliştir. Plakayı izole etmeyi dene.
- Plaka okuma için lokal model bul.

### 6. WSL ile Windows üzerinde kurulum, oto kurulum ve talimatlar
- Eğer zaman kalırsa WSL ile kurulum yap. Bu performansı arttırabilir.
- Tüm sistemin kurulumunu otomatikleştir. En baştan ROS kurup, demo çalışacak hale kadar.
- Güzel bir README yaz. Her şeyi birleştir.

### 7. Genel kod iyileştirmeleri
- Eğer hepsi bittiyse, güzel bir rewrite yapılabilir.
- Comment vs. düzelt. 
