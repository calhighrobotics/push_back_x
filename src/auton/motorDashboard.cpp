#include "main.h"
#include "robodash/api.h"
#include <vector>
#include <algorithm>
#include <functional>
#include <string>
#include "globals.h"

// --- Dashboard Configuration ---
#define TEMP_WARNING_THRESHOLD 47.0 // Degrees Celsius to trigger red color
#define HISTORY_MAX_POINTS 50       // How many points the graph holds

struct MotorDisplayData {
    std::string name;
    std::function<double()> getTemp;
    lv_obj_t* btn = nullptr;
    lv_obj_t* label = nullptr;
    std::vector<lv_coord_t> history;
    bool isOverheating = false;

    // Constructor so C++ knows how to initialize this from a list
    MotorDisplayData(std::string n, std::function<double()> f)
        : name(n), getTemp(f) {}
};

// Dashboard Globals (Local to this file)
std::vector<MotorDisplayData> motorList;
MotorDisplayData* activeMotor = nullptr; 

lv_obj_t* modal_overlay = nullptr; // Dark background
lv_obj_t* chart_modal = nullptr;
lv_obj_t* temp_chart = nullptr;
lv_chart_series_t* temp_series = nullptr;
lv_obj_t* modal_title = nullptr;

// Callback to close the modal
static void close_modal_cb(lv_event_t * e) {
    lv_obj_add_flag(chart_modal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(modal_overlay, LV_OBJ_FLAG_HIDDEN); // Hide dark background
    activeMotor = nullptr; 
}

// Callback when a motor icon is clicked
static void motor_click_cb(lv_event_t * e) {
    MotorDisplayData* md = (MotorDisplayData*)lv_event_get_user_data(e);
    activeMotor = md; 
    
    // Update title
    lv_label_set_text(modal_title, md->name.c_str());

    // Bulletproof history check
    lv_coord_t latest = md->history.empty() ? 20 : md->history.back();

    // Update dynamic chart color based on current temp
    if (latest >= TEMP_WARNING_THRESHOLD) {
        lv_chart_set_series_color(temp_chart, temp_series, lv_color_hex(0xFF4A4A)); // Red
    } else {
        lv_chart_set_series_color(temp_chart, temp_series, lv_color_hex(0x4CAF50)); // Green
    }

    // Assign the external array ONCE per click
    lv_chart_set_ext_y_array(temp_chart, temp_series, md->history.data());
    lv_chart_refresh(temp_chart);

    // Unhide the overlay and modal
    lv_obj_clear_flag(modal_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(chart_modal, LV_OBJ_FLAG_HIDDEN);
}

// Timer that runs in the background to update colors and labels
static void update_temps_cb(lv_timer_t * timer) {
    for(auto& md : motorList) {
        double rawTemp = md.getTemp();
        lv_coord_t currentTemp = (lv_coord_t)rawTemp;
        
        // Always rotate and replace the back element (constant size buffer)
        std::rotate(md.history.begin(), md.history.begin() + 1, md.history.end());
        md.history.back() = currentTemp;

        // Update Text
        char buf[32];
        snprintf(buf, sizeof(buf), "%s\n%d°C", md.name.c_str(), currentTemp);
        lv_label_set_text(md.label, buf);

        // State-tracked color update (micro-optimization)
        bool nowOverheating = (rawTemp >= TEMP_WARNING_THRESHOLD);
        if (nowOverheating != md.isOverheating) {
            md.isOverheating = nowOverheating;
            if (nowOverheating) {
                lv_obj_set_style_bg_color(md.btn, lv_color_hex(0xD32F2F), 0); // Red
            } else {
                lv_obj_set_style_bg_color(md.btn, lv_color_hex(0x2A3D8F), 0); // Blue
            }
        }
        
        // Efficiently refresh chart only if this motor's modal is open
        if (!lv_obj_has_flag(chart_modal, LV_OBJ_FLAG_HIDDEN) && activeMotor == &md) {
            // Dynamic color update on the fly
            if (currentTemp >= TEMP_WARNING_THRESHOLD) {
                lv_chart_set_series_color(temp_chart, temp_series, lv_color_hex(0xFF4A4A));
            } else {
                lv_chart_set_series_color(temp_chart, temp_series, lv_color_hex(0x4CAF50));
            }

            // Just refresh, no need to re-assign the array pointer!
            lv_chart_refresh(temp_chart);
        }
    }
}

// Initialize the dashboard UI
void init_motor_dashboard() {
    // 1. Create a proper RoboDash view
    rd_view_t* motor_view = rd_view_create("Motors");
    
    // 2. Get the parent object for this specific view (this leaves the top tabs alone!)
    lv_obj_t* parent = rd_view_obj(motor_view);
    
    // Set up a wrapping flex layout to fill the usable space
    lv_obj_set_layout(parent, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Optional: set a dark background for the view area
    lv_obj_set_style_bg_color(parent, lv_color_hex(0x1E1E1E), 0); 

    // Define the motors to track using PROS 4 singular get_temperature()
    motorList = {
        MotorDisplayData("Right\nDrive", [](){ return rightMotors.get_temperature(); }),
        MotorDisplayData("Left\nDrive", [](){ return leftMotors.get_temperature(); }),
        MotorDisplayData("Intake", [](){ return intakeMotor.get_temperature(); }),
        MotorDisplayData("Outtake", [](){ return outtakeMotor.get_temperature(); }),
        MotorDisplayData("Storage", [](){ return storageMotor.get_temperature(); })
    };

    // Build the UI for each motor
    for(auto& md : motorList) {
        md.history.assign(HISTORY_MAX_POINTS, 20);

        md.btn = lv_btn_create(parent);
        lv_obj_set_size(md.btn, 100, 100);
        lv_obj_set_style_radius(md.btn, 8, 0);
        lv_obj_set_style_bg_color(md.btn, lv_color_hex(0x2A3D8F), 0); 
        lv_obj_set_style_shadow_width(md.btn, 0, 0); 
        lv_obj_add_event_cb(md.btn, motor_click_cb, LV_EVENT_CLICKED, &md);

        md.label = lv_label_create(md.btn);
        lv_label_set_text(md.label, md.name.c_str());
        lv_obj_center(md.label);
        lv_obj_set_style_text_align(md.label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(md.label, lv_color_hex(0xFFFFFF), 0);

        lv_obj_t* vent = lv_obj_create(md.btn);
        lv_obj_set_size(vent, 50, 40);
        lv_obj_align(vent, LV_ALIGN_BOTTOM_MID, 0, -5);
        lv_obj_set_style_bg_color(vent, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_opa(vent, 100, 0); 
        lv_obj_set_style_border_width(vent, 1, 0);
        lv_obj_set_style_border_color(vent, lv_color_hex(0x000000), 0);
    }

    // Full-screen Dark Overlay (Attaches to 'parent' so it only covers the view area, not the tabs)
    modal_overlay = lv_obj_create(parent);
    lv_obj_set_size(modal_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(modal_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal_overlay, LV_OPA_50, 0); // 50% opacity
    lv_obj_set_style_border_width(modal_overlay, 0, 0);
    lv_obj_add_flag(modal_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(modal_overlay, LV_OBJ_FLAG_CLICKABLE); // Absorbs stray clicks

    // The Popup Modal (Attaches to 'parent' to stay inside the Motors view)
    chart_modal = lv_obj_create(parent);
    lv_obj_set_size(chart_modal, 400, 200);
    lv_obj_center(chart_modal);
    lv_obj_set_style_bg_color(chart_modal, lv_color_hex(0x222222), 0);
    lv_obj_set_style_border_color(chart_modal, lv_color_hex(0x555555), 0);
    lv_obj_set_style_border_width(chart_modal, 2, 0);
    lv_obj_add_flag(chart_modal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(chart_modal);

    modal_title = lv_label_create(chart_modal);
    lv_label_set_text(modal_title, "Motor Name");
    lv_obj_align(modal_title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_color(modal_title, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t* close_btn = lv_btn_create(chart_modal);
    lv_obj_set_size(close_btn, 60, 30);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, 0, -10);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_center(close_lbl);
    lv_obj_add_event_cb(close_btn, close_modal_cb, LV_EVENT_CLICKED, NULL);

    temp_chart = lv_chart_create(chart_modal);
    lv_obj_set_size(temp_chart, 360, 130);
    lv_obj_align(temp_chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_chart_set_type(temp_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(temp_chart, LV_CHART_AXIS_PRIMARY_Y, 20, 80); 
    lv_chart_set_point_count(temp_chart, HISTORY_MAX_POINTS);
    temp_series = lv_chart_add_series(temp_chart, lv_color_hex(0x4CAF50), LV_CHART_AXIS_PRIMARY_Y);

    // Start the background timer
    lv_timer_create(update_temps_cb, 1000, NULL);
}