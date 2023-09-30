#include <USlib.h>

MedianFilter<uint16_t> MnF_US_depan(MEDIAN_NUM);
MedianFilter<uint16_t> MnF_US_belakang(MEDIAN_NUM);
MedianFilter<uint16_t> MnF_US_kanan(MEDIAN_NUM);
MedianFilter<uint16_t> MnF_US_kiri(MEDIAN_NUM);

MeanFilter<uint16_t> MF_US_depan(MEDIAN_NUM);
MeanFilter<uint16_t> MF_US_belakang(MEDIAN_NUM);
MeanFilter<uint16_t> MF_US_kanan(MEDIAN_NUM);
MeanFilter<uint16_t> MF_US_kiri(MEDIAN_NUM);


void USlib::update(){
    // for (size_t i = 0; i < SONAR_NUM; i++)
    // {
    //     /* code */
    //     US_cm[i] = sonar[i].convert_cm(sonar[i].ping_median(MEDIAN_NUM));
    // }



    

    // MnF_US_kanan.AddValue(sonar[2].ping_cm());
    // US_kanan = MnF_US_kanan.GetFiltered();

    // MnF_US_kiri.AddValue(sonar[3].ping_cm());
    // US_kiri = MnF_US_kiri.GetFiltered(); 

    US_depan        = sonar[0].ping_cm();
    US_depan        = MnF_US_depan.AddValue(US_depan);
    US_depan        = MF_US_depan.AddValue(US_depan);

    US_belakang     = sonar[1].ping_cm();
    US_belakang     = MnF_US_belakang.AddValue(US_belakang);
    US_belakang     = MF_US_belakang.AddValue(US_belakang);

    US_kanan        = sonar[2].ping_cm();
    US_kanan        = MnF_US_kanan.AddValue(US_kanan);
    US_kanan        = MF_US_kanan.AddValue(US_kanan);

    US_kiri         = sonar[3].ping_cm();
    US_kiri         = MnF_US_kiri.AddValue(US_kiri);
    US_kiri         = MF_US_kiri.AddValue(US_kiri);
}
