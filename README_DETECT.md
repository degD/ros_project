# KURULUM

Kullanıcının Ubuntu 20.04 bir sistemi olduğu, yeterli depolama alanına sahip olduğu, modern bir CUDA destekli GPU kullandığı, CUDA sürümünün 11.8'den yüksek olduğu var sayılıyor.

```
mkdir -p mmlab
cd mmlab
conda create -n mmlab python=3.9 -y
conda activate mmlab
conda install pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 pytorch-cuda=11.8 -c pytorch -c nvidia
pip install -U openmim
pip install fsspec
mim install mmcv-full==1.7.2
mim install mmdet\<3.0.0
pip install --force-reinstall -v "numpy==1.25.2"
pip install mmrotate
mim download mmrotate --config oriented_rcnn_r50_fpn_1x_dota_le90 --dest .
python test.py 2> /dev/null
```

Eğer kurulum sorunsuz gerçekleştiyse ekranda koordinatlar içeren diziler göreceksiniz. Eğer bir hatayla karşılaşırsanız, hiçbir çıktı görmezseniz, adımlarınızı tekrar gözden geçirin. Yine bir çözüme ulaşamıyorsanız bir **issue oluşturarak** yardım isteyebilirsiniz.

## Çalışma Sistemi
1. Drone navigasyonu tamamlamış ve fotoğrafları çekmiş. Fotoğraflar `PHOTOS_DIR` klasöründe.
2. Her fotoğraf adı, orta noktanın koordinatlarını veriyor. Araç tespiti de resme göre araç konumlarını.
3. Bu iki veri kullanılarak araçların haritaya göre koordinatları hesaplanabilir. Buna küresel koordinat diyeceğim.
4. Burada elbette yanlış ve eksik işaretlemeler olacak. Küresel koordinatlar ile doğruluk oranları da tutulabilir.
5. Koordinatlar kaç kez araç olarak tespit edilmiş, hangi doğruluk oranları ile, bakılarak bazıları gözden çıkarılabilir.
6. Kalanlarda da drone yaklaştığında sonuç değişirse araç olmadığı anlaşılır (Bu modülün dışında).
7. Araç olduğu anlaşılanlar için Plaka okuma kısmına geçilir.

## İlerleme
- 4 numaralı adıma kadar tamamlandı. 4 kısmen tamamlandı. Koordinatlar dönüştürülüyor ancak doğrulama tam değil.

## Hata Çözüm
-Eğer fotoğraf çekilirken hata veriyorsa: `export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5` komutunu çalıştırın. Sonra yeniden deneyin.
- Uzaktan fotoğraflarda doğruluk oranı çok daha yüksek. Fotoğrafları yüksekten çek.