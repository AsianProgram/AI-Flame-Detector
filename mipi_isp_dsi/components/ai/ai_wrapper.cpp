#include "coco_detect.hpp"
#include "ai_wrapper.h" 

static COCODetect *detector = nullptr;

extern "C" {

void ai_init(void)
{
    if (!detector) {
        detector = new COCODetect(COCODetect::YOLO11N_CUSTOM_320, false);
    }
}

void ai_run(uint8_t *image_buf, int width, int height, detection_t *output)
{

    if (!detector) return;

    detection_t result = {0};
    float best_score = -1;

    dl::image::img_t img = {
        .data = image_buf,
        .width = static_cast<uint16_t>(width),
        .height = static_cast<uint16_t>(height),
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565
    };
    auto &results = detector->run(img);

    for (auto &res : results) {
        if (res.score > best_score) {
        best_score = res.score;
    
        // ESP_LOGI("AI",
        //          "cat=%d score=%.2f box=[%d,%d,%d,%d]",
        //          res.category,
        //          res.score,
        //          res.box[0],
        //          res.box[1],
        //          res.box[2],
        //          res.box[3]);
        result.category = res.category;
        result.score = res.score;
        result.x1 = res.box[0];
        result.y1 = res.box[1];
        result.x2 = res.box[2];
        result.y2 = res.box[3];
        }
    }

    *output = result;
}
// void ai_run(uint8_t *image_buf, int width, int height)
// {
//     if (!detector) return;

//     dl::image::img_t img = {
//         .data = image_buf,
//         .width = static_cast<uint16_t>(width),
//         .height = static_cast<uint16_t>(height),
//         .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565
//     };
//     auto &results = detector->run(img);

//     for (auto &res : results) {
//         ESP_LOGI("AI",
//                  "cat=%d score=%.2f box=[%d,%d,%d,%d]",
//                  res.category,
//                  res.score,
//                  res.box[0],
//                  res.box[1],
//                  res.box[2],
//                  res.box[3]);
//     }
// }

}
