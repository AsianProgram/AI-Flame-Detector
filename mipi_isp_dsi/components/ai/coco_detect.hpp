#pragma once
#include "dl_detect_base.hpp"
#include "dl_detect_yolo11_postprocessor.hpp"

namespace coco_detect {
class Yolo11n : public dl::detect::DetectImpl {
public:
    static inline constexpr float default_score_thr = 0.2;
    static inline constexpr float default_nms_thr = 0.45;
    Yolo11n(const char *model_name, float score_thr, float nms_thr);
};  //orginal is score=0.25, nms=0.7
} // namespace coco_detect

class COCODetect : public dl::detect::DetectWrapper {
public:
    typedef enum {
        YOLO11N_S8_V1,
        YOLO11N_S8_V2,
        YOLO11N_S8_V3,
        YOLO11N_320_S8_V3,
        YOLO11N_CUSTOM_320,
        YOLO11N_TEST
    } model_type_t;
    //COCODetect(model_type_t model_type = static_cast<model_type_t>(CONFIG_DEFAULT_COCO_DETECT_MODEL), bool lazy_load = true);
    COCODetect(model_type_t model_type = COCODetect::YOLO11N_CUSTOM_320,
           bool lazy_load = true);
private:
    void load_model() override;

    model_type_t m_model_type;
};
