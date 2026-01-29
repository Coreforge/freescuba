#include "app_state.hpp"
#include "cal_poses.h"
#include "glove_model.hpp"
#include "imgui_extensions.hpp"
#define _USE_MATH_DEFINES
#include <openvr.h>

#include "user_interface.hpp"
#include <cmath>

#include "maths.hpp"
#include "utils.h"

// DrawJoystickInput :: Imgui commands to draw a nice joystick control to show what the current joystick value is
// DrawGlove :: Displays a single Glove's state. Contains the buttons for trigger state change.

ImFont* fontBold    = nullptr;
ImFont* fontLight   = nullptr;
ImFont* fontRegular = nullptr;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(t,a,b) (MAX(MIN(t, b), a))

void SetupImgui() {
    // @TODO: ImGui style here

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    ImGuiStyle& style = ImGui::GetStyle();

    // Main window should not have any border as it's rooted to the executable window
    style.WindowBorderSize  = 0.0f;
    style.WindowRounding    = 0.0f;

    // Make buttons bigger so that they have a bigger hitbox in VR
    style.FramePadding      = ImVec2(10, 6);
    style.ItemSpacing       = ImVec2(10, 6);

    // Rounding
    style.ChildRounding     = 6.0f;
    style.FrameRounding     = 6.0f;
    style.PopupRounding     = 6.0f;
    style.TabRounding       = 6.0f;

    // Colours
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_WindowBg]           = ImVec4(0.01f, 0.01f, 0.01f, 0.94f);
    colors[ImGuiCol_Border]             = ImVec4(0.30f, 0.30f, 0.30f, 0.55f);
    colors[ImGuiCol_FrameBg]            = ImVec4(0.24f, 0.26f, 0.27f, 0.54f);
    colors[ImGuiCol_FrameBgHovered]     = ImVec4(0.26f, 0.26f, 0.26f, 0.40f);
    colors[ImGuiCol_FrameBgActive]      = ImVec4(0.31f, 0.31f, 0.31f, 0.67f);
    colors[ImGuiCol_SliderGrab]         = ImVec4(0.26f, 0.43f, 0.68f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]   = ImVec4(0.36f, 0.56f, 0.86f, 1.00f);
    colors[ImGuiCol_Button]             = ImVec4(0.26f, 0.43f, 0.68f, 1.00f);
    colors[ImGuiCol_ButtonHovered]      = ImVec4(0.29f, 0.52f, 0.85f, 1.00f);
    colors[ImGuiCol_ButtonActive]       = ImVec4(0.21f, 0.33f, 0.52f, 1.00f);

    const float FONT_SIZE = 16.0f;

    #ifdef WIN32
    fontRegular = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf",  FONT_SIZE); // Segoe UI
    fontBold    = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeuib.ttf", FONT_SIZE); // Segoe UI Bold
    fontLight   = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeuil.ttf", FONT_SIZE); // Segoe UI Light
    #endif
}
void CleanupImgui() {

}

void DrawJoystickInput(const float valueX, const float valueY, const float deadzone) {
    const float deadzoneRemapped = deadzone * 100.f;
    const ImU32 whiteColor = IM_COL32(255, 255, 255, 255);
    constexpr float BOX_SIZE = 200;
    constexpr float CIRCLE_RADIUS = 10;
    const ImVec2 posR = ImGui::GetCursorPosAbsolute();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    // Safe region
    drawList->AddRectFilled(posR, ImVec2(posR.x + BOX_SIZE, posR.y + BOX_SIZE), IM_COL32(30,0,0,255));
    drawList->AddCircleFilled(ImVec2(posR.x + BOX_SIZE * 0.5f, posR.y + BOX_SIZE * 0.5f), deadzoneRemapped, IM_COL32(0,0,0,255), 32);

    // Frame and cross lines
    drawList->AddRect(posR, ImVec2(posR.x + 200, posR.y + 200), whiteColor);
    drawList->AddLine(ImVec2(posR.x + 100, posR.y), ImVec2(posR.x + 100, posR.y + 200), whiteColor);
    drawList->AddLine(ImVec2(posR.x, posR.y + 100), ImVec2(posR.x + 200, posR.y + 100), whiteColor);
    // Draw threshold and unit radius circles.
    drawList->AddCircle(ImVec2(posR.x + 100, posR.y + 100), deadzoneRemapped, IM_COL32(0, 255, 0, 255), 32);
    drawList->AddCircle(ImVec2(posR.x + 100, posR.y + 100), 100, whiteColor, 32);
    // Current axis position.
    drawList->AddCircleFilled(ImVec2(posR.x + valueX * 100 + 100, posR.y + valueY * 100 + 100), CIRCLE_RADIUS, IM_COL32(255, 255, 255, 191));
    drawList->AddCircleFilled(ImVec2(posR.x + valueX * 100 + 100, posR.y + valueY * 100 + 100), 1.1f, IM_COL32(255, 0, 0, 255));

    ImGui::Dummy(ImVec2(BOX_SIZE, BOX_SIZE));
}

void DrawGlove(const std::string name, const std::string id, protocol::ContactGloveState_t& glove, AppState& state) {

    std::string panelTitle = name;
    if (glove.isConnected) {
        panelTitle = name + " (" + std::to_string(CLAMP(glove.gloveBattery, 0, 100)) + "%)";
    }
    ImGui::BeginGroupPanel(panelTitle.c_str());
    {
        if (!glove.isConnected) {
            ImGui::TextDisabled("Glove not connected...");
        } else {
            ImGui::PushID((id + "_magnetra_group").c_str());
            ImGui::BeginGroupPanel("Magnetra Settings");
            {
                ImGui::Spacing();
                ImGui::Spacing();

                if (!glove.hasMagnetra) {
                    ImGui::TextDisabled("Magnetra is not available... Inputs unavailable.");
                } else {
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Joystick");
                    ImGui::PopFont();
                    ImGui::SameLine();
                    ImGui::Text(": (%.4f, %.4f)", glove.joystickX, glove.joystickY);
                    ImGui::Spacing();
                    DrawJoystickInput(glove.joystickXUnfiltered, glove.joystickYUnfiltered, glove.calibration.joystick.threshold);

                    ImGui::Spacing();
                    ImGui::Spacing();

                    ImGui::Text("Deadzone");
                    ImGui::SameLine();
                    ImGui::PushID((id + "_joystick_threshold").c_str());
                    ImGui::SliderFloat("##threshold", &glove.calibration.joystick.threshold, 0.f, 1.0f);
                    ImGui::PopID();

                    ImGui::Spacing();
                    ImGui::Spacing();

                    ImGui::PushFont(fontBold);
                    ImGui::Text("Buttons");
                    ImGui::PopFont();

                    float columnWidth = ImGui::CalcItemWidth() / 5.0f;
                    ImGui::PushID((id + "_buttons_container").c_str());
                    ImGui::Columns(7);
                    for (int i = 0; i < 7; i++) {
                        ImGui::SetColumnWidth(i, columnWidth);
                    }
                    ImGui::PushID((id + "_button_a").c_str());
                    ImGui::RadioButton("A##gloveL", glove.buttonUp);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id + "_button_b").c_str());
                    ImGui::RadioButton("B##gloveL", glove.buttonDown);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id + "_system_up").c_str());
                    ImGui::RadioButton("Sys Up##gloveL", glove.systemUp);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id + "_system_down").c_str());
                    ImGui::RadioButton("Sys Down##gloveL", glove.systemDown);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id + "_joystick_click").c_str());
                    ImGui::RadioButton("Joystick Click##gloveL", glove.joystickClick);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id+"_trigger_click").c_str());
                    ImGui::RadioButton("Trigger Click##gloveL", glove.triggerClick);
                    ImGui::PopID();
                    ImGui::NextColumn();
                    ImGui::PushID((id+"_system_btn").c_str());
                    ImGui::RadioButton("System Button##gloveL", glove.systemButton);
                    ImGui::PopID();
                    ImGui::Columns(1);
                    ImGui::PopID();

                    ImGui::PushFont(fontBold);
                    ImGui::Text("Trigger");
                    ImGui::PopFont();
                    ImGui::SameLine();
                    ImGui::ProgressBar(glove.trigger, ImVec2(-FLT_MIN, 0), nullptr);
                    

                    ImGui::Spacing();
                    ImGui::Spacing();

                    if (ImGui::Button("Calibrate Thumbstick")) {
                        state.uiState.page = ScreenState_t::ScreenStateCalibrateJoystick;
                        state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    }
                    if (ImGui::Button("Calibrate Trigger")) {
                        state.uiState.page = ScreenState_t::ScreenStateCalibrateTrigger;
                        state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    }

                    // @TODO: Adjust forward angle for fine tuning
                }

                ImGui::Spacing();
            }
            ImGui::EndGroupPanel();
            ImGui::PopID();

            ImGui::PushID((id + "_fingers_group").c_str());
            ImGui::BeginGroupPanel("Finger Tracking");
            {
                ImGui::Spacing();
                ImGui::Spacing();
                if (ImGui::BeginTable((id + "_fingers_table").c_str(), 5, ImGuiTableFlags_SizingFixedFit))
                {
                    float windowWidth = ImGui::GetWindowWidth();
                    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 0.0f);
                    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 0.0f);
                    float widthBars = (windowWidth - ImGui::GetColumnWidth(0) - ImGui::GetColumnWidth(1)) * 0.33f;
                    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, widthBars);
                    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, widthBars);
                    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, widthBars);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Thumb Base");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f)", glove.thumbBase);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.thumbBase);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Thumb");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f, %.4f, %.4f)", glove.thumbRoot, glove.thumbTip, glove.thumbSplay);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.thumbRoot);
                    ImGui::TableSetColumnIndex(3);
                    DRAW_FINGER_BEND_VALUE(glove.thumbTip);
                    ImGui::TableSetColumnIndex(4);
                    DRAW_FINGER_BEND_VALUE(glove.thumbSplay);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Index finger");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f, %.4f, %.4f)", glove.indexRoot, glove.indexTip, glove.indexSplay);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.indexRoot);
                    ImGui::TableSetColumnIndex(3);
                    DRAW_FINGER_BEND_VALUE(glove.indexTip);
                    ImGui::TableSetColumnIndex(4);
                    DRAW_FINGER_BEND_VALUE(glove.indexSplay);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Middle finger");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f, %.4f, %.4f)", glove.middleRoot, glove.middleTip, glove.middleSplay);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.middleRoot);
                    ImGui::TableSetColumnIndex(3);
                    DRAW_FINGER_BEND_VALUE(glove.middleTip);
                    ImGui::TableSetColumnIndex(4);
                    DRAW_FINGER_BEND_VALUE(glove.middleSplay);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Ring finger");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f, %.4f, %.4f)", glove.ringRoot, glove.ringTip, glove.ringSplay);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.ringRoot);
                    ImGui::TableSetColumnIndex(3);
                    DRAW_FINGER_BEND_VALUE(glove.ringTip);
                    ImGui::TableSetColumnIndex(4);
                    DRAW_FINGER_BEND_VALUE(glove.ringSplay);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushFont(fontBold);
                    ImGui::Text("Pinky finger");
                    ImGui::PopFont();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.4f, %.4f, %.4f)", glove.pinkyRoot, glove.pinkyTip, glove.pinkySplay);
                    ImGui::TableSetColumnIndex(2);
                    DRAW_FINGER_BEND_VALUE(glove.pinkyRoot);
                    ImGui::TableSetColumnIndex(3);
                    DRAW_FINGER_BEND_VALUE(glove.pinkyTip);
                    ImGui::TableSetColumnIndex(4);
                    DRAW_FINGER_BEND_VALUE(glove.pinkySplay);

                    ImGui::EndTable();
                }

                ImGui::Spacing(); 
                ImGui::Spacing();
                if (ImGui::Button("Calibrate finger bounds")) {
                    state.uiState.page = ScreenState_t::ScreenStateCalibrateSensorBounds;
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                }

                if (ImGui::Button("Calibrate fingers")) {
                    state.uiState.page = ScreenState_t::ScreenStateCalibrateFingers;
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                }

                if (ImGui::Button("Calibrate single finger")) {
                    state.uiState.page = ScreenState_t::ScreenStateCalibrateFingersSingle;
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                }

                ImGui::Checkbox("Use finger curl", &glove.useCurl);

                ImGui::Spacing();

                ImGui::PushID((id + "_fingers_splay_group").c_str());
                ImGui::BeginGroupPanel("Splay");
                {
                    if (ImGui::BeginTable((id + "_fingers_splay_group").c_str(), 3, ImGuiTableFlags_SizingFixedFit)) {
                        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 0.0f);   
                        ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, 0.0f);
                        ImGui::TableSetupColumn("Offset", ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, 0.0f);

                        ImGui::TableNextRow();

#define DRAW_SPLAY_SLIDERS(finger, label) \
                        ImGui::TableSetColumnIndex(0); \
                        ImGui::PushFont(fontBold); \
                        ImGui::Text(label); \
                        ImGui::PopFont(); \
                        ImGui::TableSetColumnIndex(1); \
                        ImGui::SliderFloat((std::string("##") + label" scale").c_str(), &glove.calibration.splay.finger.scale, 0.f, 2.f); \
                        ImGui::TableSetColumnIndex(2); \
                        ImGui::SliderFloat((std::string("##") + label" offset").c_str(), &glove.calibration.splay.finger.offset, -1.f, 1.f);

                        DRAW_SPLAY_SLIDERS(thumb, "Thumb")
                        ImGui::TableNextRow();
                        DRAW_SPLAY_SLIDERS(index, "Index")
                        ImGui::TableNextRow();
                        DRAW_SPLAY_SLIDERS(middle, "Middle")
                        ImGui::TableNextRow();
                        DRAW_SPLAY_SLIDERS(ring, "Ring")
                        ImGui::TableNextRow();
                        DRAW_SPLAY_SLIDERS(pinky, "Pinky")

                        ImGui::EndTable();
                    }
                }
                ImGui::EndGroupPanel();
                ImGui::PopID();

                ImGui::Spacing();

                ImGui::PushID((id + "_fingers_gestures_group").c_str());
                ImGui::BeginGroupPanel("Gestures");
                {
                    ImGui::Spacing();
                    ImGui::Spacing();

                    if (ImGui::BeginTable((id + "_fingers_slider_group").c_str(), 3, ImGuiTableFlags_SizingFixedFit)) {
                        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 0.0f);   
                        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize, 0.0f);

                        ImGui::TableNextRow();
                        DRAW_ALIGNED_SLIDER_FLOAT("Thumb Activate", &glove.calibration.gestures.thumb.activate, 0, 1);
                        ImGui::TableNextRow();
                        DRAW_ALIGNED_SLIDER_FLOAT("Thumb Deactivate", &glove.calibration.gestures.thumb.deactivate, 0, 1);
                        ImGui::TableNextRow();

                        DRAW_ALIGNED_SLIDER_FLOAT("Trigger Activate", &glove.calibration.gestures.trigger.activate, 0, 1);
                        ImGui::TableNextRow();
                        DRAW_ALIGNED_SLIDER_FLOAT("Trigger Deactivate", &glove.calibration.gestures.trigger.deactivate, 0, 1);
                        ImGui::TableNextRow();

                        DRAW_ALIGNED_SLIDER_FLOAT("Grip Activate", &glove.calibration.gestures.grip.activate, 0, 1);
                        ImGui::TableNextRow();
                        DRAW_ALIGNED_SLIDER_FLOAT("Grip Deactivate", &glove.calibration.gestures.grip.deactivate, 0, 1);

                        ImGui::EndTable();
                    }

                    ImGui::Spacing();
                    ImGui::Text("Grip");
                    DRAW_FINGER_BEND_VALUE(glove.gestureGrip);
                }
                
                if (ImGui::CollapsingHeader("Raw values")) {
#define FINGER_ROOT(label, finger) \
                    ImGui::Text(label " Root:  %d %d %.4f %d", glove.finger##Root1Raw, glove.finger##Root2Raw, (float)glove.finger##Root1Raw/glove.finger##Root2Raw, glove.finger##Root1Raw + glove.finger##Root2Raw)
                    ImGui::Text("Thumb Base:   %d", glove.thumbBaseRaw);
                    FINGER_ROOT("Thumb", thumb);
                    //ImGui::Text("Thumb Root:  %d %d %.4f %d %d", glove.thumbRoot1Raw, glove.thumbRoot2Raw, (float)glove.thumbRoot1Raw/glove.thumbRoot2Raw);
                    ImGui::Text("Thumb Tip:   %d", glove.thumbTipRaw);
                    //ImGui::Text("Index Root:  %d %d %.4f %d %d", glove.indexRoot1Raw, glove.indexRoot2Raw, (float)glove.indexRoot1Raw/glove.indexRoot2Raw);
                    FINGER_ROOT("Index", index);
                    ImGui::Text("Index Tip:   %d", glove.indexTipRaw);
                    //ImGui::Text("Middle Root: %d %d %.4f %d %d", glove.middleRoot1Raw, glove.middleRoot2Raw, (float)glove.middleRoot1Raw/glove.middleRoot2Raw);
                    FINGER_ROOT("Middle", middle);
                    ImGui::Text("Middle Tip:  %d", glove.middleTipRaw);
                    //ImGui::Text("Ring Root:   %d %d %.4f %d %d", glove.ringRoot1Raw, glove.ringRoot2Raw, (float)glove.ringRoot1Raw/glove.ringRoot2Raw);
                    FINGER_ROOT("Ring", ring);
                    ImGui::Text("Ring Tip:    %d", glove.ringTipRaw);
                    //ImGui::Text("Pinky Root:  %d %d %.4f %d %d", glove.pinkyRoot1Raw, glove.pinkyRoot2Raw, (float)glove.pinkyRoot1Raw/glove.pinkyRoot2Raw);
                    FINGER_ROOT("Pinky", pinky);
                    ImGui::Text("Pinky Tip:   %d", glove.pinkyTipRaw);
                }

                ImGui::EndGroupPanel();
                ImGui::PopID();
            }
            ImGui::EndGroupPanel();
            ImGui::PopID();

            ImGui::PushFont(fontBold);
            ImGui::Text("Misc");
            ImGui::PopFont();
            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::TextDisabled("Battery: ");
            ImGui::SameLine();
            ImGui::Text((std::to_string(glove.gloveBattery) + "%%").c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("    Version: ");
            ImGui::SameLine();
            ImGui::Text((std::to_string(glove.firmwareMajor) + "." + std::to_string(glove.firmwareMinor)).c_str());

            {
                ImGui::PushID((id + "_offset_adjust_group").c_str());
                // Disallow tracker calibration if a tracker is not available
                ImGui::BeginDisabled(glove.trackerIndex == CONTACT_GLOVE_INVALID_DEVICE_ID);
                #ifdef WIN32
                if (ImGui::Button("Calibrate offset")) {
                    state.uiState.page = ScreenState_t::ScreenStateCalibrateOffset;
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                }
                #endif

                ImGui::SameLine();

                if (ImGui::Button("Reset offset")) {
                    glove.calibration.poseOffset.pos[0] = 0;
                    glove.calibration.poseOffset.pos[1] = 0;
                    glove.calibration.poseOffset.pos[2] = 0;

                    glove.calibration.poseOffset.rot[0] = 0.0f;
                    glove.calibration.poseOffset.rot[1] = 0.0f;
                    glove.calibration.poseOffset.rot[2] = 0.0f;
                    glove.calibration.poseOffset.rot[3] = 1.0f;
                }

                ImGui::EndDisabled();

                ImGui::Spacing();
                ImGui::PushFont(fontBold);
                ImGui::Text("Position Offset");
                ImGui::PopFont();
                ImGui::Spacing();

                ImGui::DrawVectorElement(id + "_tracker_position_offset", "X", &glove.calibration.poseOffset.pos[0]);
                ImGui::DrawVectorElement(id + "_tracker_position_offset", "Y", &glove.calibration.poseOffset.pos[1]);
                ImGui::DrawVectorElement(id + "_tracker_position_offset", "Z", &glove.calibration.poseOffset.pos[2]);

                ImGui::Spacing();
                ImGui::PushFont(fontBold);
                ImGui::Text("Rotation Offset");
                ImGui::PopFont();
                ImGui::Spacing();

                ImGui::DrawVectorElement(id + "_tracker_rotation_offset", "X", &glove.calibration.poseOffset.rot[0]);
                ImGui::DrawVectorElement(id + "_tracker_rotation_offset", "Y", &glove.calibration.poseOffset.rot[1]);
                ImGui::DrawVectorElement(id + "_tracker_rotation_offset", "Z", &glove.calibration.poseOffset.rot[2]);
                ImGui::DrawVectorElement(id + "_tracker_rotation_offset", "W", &glove.calibration.poseOffset.rot[3]);

                ImGui::Spacing();
                ImGui::Checkbox("Add grip offset", &glove.calibration.use_custom_grip);
                ImGui::PushFont(fontBold);
                ImGui::Text("Grip Position Offset");
                ImGui::PopFont();
                ImGui::Spacing();

                ImGui::DrawVectorElement(id + "_grip_position_offset", "X", &glove.calibration.grip_offset[0]);
                ImGui::DrawVectorElement(id + "_grip_position_offset", "Y", &glove.calibration.grip_offset[1]);
                ImGui::DrawVectorElement(id + "_grip_position_offset", "Z", &glove.calibration.grip_offset[2]);

                ImGui::Spacing();
                ImGui::PushFont(fontBold);
                ImGui::Text("Grip Rotation Offset");
                ImGui::PopFont();
                ImGui::Spacing();

                ImGui::DrawVectorElement(id + "_grip_rotation_offset", "X", &glove.calibration.grip_rotation[0]);
                ImGui::DrawVectorElement(id + "_grip_rotation_offset", "Y", &glove.calibration.grip_rotation[1]);
                ImGui::DrawVectorElement(id + "_grip_rotation_offset", "Z", &glove.calibration.grip_rotation[2]);

                ImGui::PopID();
            }
            ImGui::Spacing();
        }
    }
    ImGui::EndGroupPanel();
}

void DrawCalibrateTrigger(AppState& state){
    ImGui::BeginGroupPanel("Trigger Calibration");
    // most of this is copied from joystick calibration, which is quite similar
    {
    // Isolate the glove we wish to work on
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Trigger...");
            desiredGlove = &state.gloveLeft;
        }
        else {
            ImGui::Text("Calibrating Right Trigger...");
            desiredGlove = &state.gloveRight;
        }

        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            // Create a new calibration state, and 0 the joystick calibration
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            memset(&state.uiState.currentCalibration.trigger, 0, sizeof(protocol::ContactGloveState_t::CalibrationData_t::TriggerCalibration_t));

            state.uiState.currentCalibration.trigger.min = 0xff;
            state.uiState.currentCalibration.trigger.max = 0x00;

            state.uiState.calibrationState = CalibrationState_t::Trigger_DiscoverNeutral;
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedNoJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown;

        switch (state.uiState.calibrationState) {
            case CalibrationState_t::Trigger_DiscoverNeutral:
            {
                ImGui::Text("Let the trigger rest");
                state.uiState.currentCalibration.trigger.min = MIN(state.uiState.currentCalibration.trigger.min, desiredGlove->triggerRaw);

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedNoJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Trigger_DiscoverClick;
                }
                break;
            }
            case CalibrationState_t::Trigger_DiscoverClick:
            {
                ImGui::Text("Fully pull the trigger until it clicks");
                state.uiState.currentCalibration.trigger.max = MAX(state.uiState.currentCalibration.trigger.max, desiredGlove->triggerRaw);

                // Proceed on any input
                ImGui::Text("Press any button to continue... (or click the trigger)");

                if (ImGui::Button("Continue") || anyButtonPressedNoJoystick || desiredGlove->triggerClick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    state.uiState.page = ScreenState_t::ScreenStateViewData;

                    // Copy the new calibration back
                    memcpy(&desiredGlove->calibration , &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
                }
                break;
            }
            default:
            {
                ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
                break;
            }
        }    
        if (ImGui::Button("Cancel")) {
            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;

            // Set the old calibration
            // Should not be necessary
            // memcpy(&desiredGlove->calibration, &state.uiState.oldCalibration, sizeof(protocol::ContactGloveState::CalibrationData));
        }        
    }
    ImGui::EndGroupPanel();
}

void DrawCalibrateJoystick(AppState& state) {
    ImGui::BeginGroupPanel("Joystick Calibration");
    {
        // @TODO: Style the UI to look pretty

        // Isolate the glove we wish to work on
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Joystick...");
            desiredGlove = &state.gloveLeft;
        }
        else {
            ImGui::Text("Calibrating Right Joystick...");
            desiredGlove = &state.gloveRight;
        }

        // Handled here so that we don't get a blank frame
        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            // Create a new calibration state, and 0 the joystick calibration
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            memset(&state.uiState.currentCalibration.joystick, 0, sizeof(protocol::ContactGloveState_t::CalibrationData_t::JoystickCalibration_t));
            
            // Set intial values for calibration
            state.uiState.currentCalibration.joystick.XMax		= 0x0000;
            state.uiState.currentCalibration.joystick.XMin		= 0xFFFF;
            state.uiState.currentCalibration.joystick.YMax		= 0x0000;
            state.uiState.currentCalibration.joystick.YMin		= 0xFFFF;
            state.uiState.currentCalibration.joystick.XNeutral	= 0x0000;
            state.uiState.currentCalibration.joystick.YNeutral	= 0xFFFF;
            state.uiState.joystickForwardX						= 0x0000;
            state.uiState.joystickForwardY						= 0x0000;
            state.uiState.currentCalibration.joystick.forwardAngle		= 0.0f;

            // Copy user settings
            state.uiState.currentCalibration.joystick.threshold = desiredGlove->calibration.joystick.threshold;

            // Done. Move to DiscoverBounds
            state.uiState.calibrationState = CalibrationState_t::Joystick_DiscoverBounds;
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedNoJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown;
        
        // Calibration code
        switch (state.uiState.calibrationState) {
            case CalibrationState_t::Joystick_DiscoverBounds:
                {
                    ImGui::Text("Move the joystick in all directions for a few seconds...");

                    // Discover min max for each joystick
                    state.uiState.currentCalibration.joystick.XMax = MAX(state.uiState.currentCalibration.joystick.XMax, desiredGlove->joystickXRaw);
                    state.uiState.currentCalibration.joystick.XMin = MIN(state.uiState.currentCalibration.joystick.XMin, desiredGlove->joystickXRaw);

                    state.uiState.currentCalibration.joystick.YMax = MAX(state.uiState.currentCalibration.joystick.YMax, desiredGlove->joystickYRaw);
                    state.uiState.currentCalibration.joystick.YMin = MIN(state.uiState.currentCalibration.joystick.YMin, desiredGlove->joystickYRaw);

                    // Proceed on any input
                    ImGui::Text("Press any button to continue...");

                    if (ImGui::Button("Continue") || anyButtonPressedNoJoystick) {
                        // Move to the next state
                        state.uiState.calibrationState = CalibrationState_t::Joystick_DiscoverNeutral;
                    }

                    break;
                }
            case CalibrationState_t::Joystick_DiscoverNeutral:
                {
                    // Discover neutral for joystick
                    ImGui::Text("Do not move the joystick, let it settle.");

                    state.uiState.currentCalibration.joystick.XNeutral = desiredGlove->joystickXRaw;
                    state.uiState.currentCalibration.joystick.YNeutral = desiredGlove->joystickYRaw;

                    // Proceed on any input
                    ImGui::Text("Press any button to continue...");

                    if (ImGui::Button("Continue") || anyButtonPressedNoJoystick) {
                        // Move to the next state
                        state.uiState.calibrationState = CalibrationState_t::Joystick_DiscoverForward;
                    }

                    break;
                }
            case CalibrationState_t::Joystick_DiscoverForward:
                {
                    // Discover the forward direction of the joystick, compute an output angle from this
                    ImGui::Text("Move the joystick forward.");

                    state.uiState.joystickForwardX = desiredGlove->joystickXRaw;
                    state.uiState.joystickForwardY = desiredGlove->joystickYRaw;

                    // Proceed on any input
                    ImGui::Text("Press any button to continue...");

                    if (ImGui::Button("Continue") || anyButtonPressedNoJoystick) {
                        // Compute the forward vector
                        float joystickX = 2.0f * (state.uiState.joystickForwardX - state.uiState.currentCalibration.joystick.XMin) / (float)MAX(state.uiState.currentCalibration.joystick.XMax - state.uiState.currentCalibration.joystick.XMin, 0.0f) - 1.0f;
                        float joystickY = 2.0f * (state.uiState.joystickForwardY - state.uiState.currentCalibration.joystick.YMin) / (float)MAX(state.uiState.currentCalibration.joystick.YMax - state.uiState.currentCalibration.joystick.YMin, 0.0f) - 1.0f;

                        // Normalize the axis if out of range
                        if (joystickX * joystickX + joystickY * joystickY > 1.0f) {
                            const double length = sqrt(joystickX * joystickX + joystickY * joystickY);
                            joystickX = (float) (joystickX / length);
                            joystickY = (float) (joystickY / length);
                        }

                        // Get the angle it's at, and store in the calibration
                        const float angle = atan2(joystickX , -joystickY);
                        state.uiState.currentCalibration.joystick.forwardAngle = angle; // Negate the angle, and rotate -90 degrees

                        // Move to the next state
                        state.uiState.calibrationState = CalibrationState_t::State_Entering;
                        state.uiState.page = ScreenState_t::ScreenStateViewData;

                        // Copy the new calibration back
                        memcpy(&desiredGlove->calibration , &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
                    }

                    break;
                }
            default:
                {
                    ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
                    break;
                }
        }

        if (ImGui::Button("Cancel")) {
            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;

            // Set the old calibration
            // Should not be necessary
            // memcpy(&desiredGlove->calibration, &state.uiState.oldCalibration, sizeof(protocol::ContactGloveState::CalibrationData));
        }
    }
    ImGui::EndGroupPanel();
}

void DrawCalibrateFingers(AppState& state) {

    ImGui::BeginGroupPanel("Finger Calibration");
    {
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        GloveModelSolver* solver;
        std::vector<RecordedCalibrationPose>* calPoses;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Glove Fingers...");
            desiredGlove = &state.gloveLeft;
            solver = &state.solverLeft;
            calPoses = &state.calPosesLeft;
        }
        else {
            ImGui::Text("Calibrating Right Glove Fingers...");
            desiredGlove = &state.gloveRight;
            solver = &state.solverRight;
            calPoses = &state.calPosesRight;
        }

        // Handled here so that we don't get a blank frame
        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));

            // Create a new calibration state, and 0 out the finger calibration
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            // since bounds aren't being rediscovered right now and are stored in the same place, they would be lost
            //memset(&state.uiState.currentCalibration.fingers, 0, sizeof(protocol::ContactGloveState_t::FingerCalibrationData_t));
            
            // Done. Move to DiscoverBounds
            state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown || state.uiState.gloveButtons.releasedLeft.joystickClick ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown || state.uiState.gloveButtons.releasedRight.joystickClick;

        #define COPY_FINGER_STATE(finger, jointstate) \
        state.uiState.currentCalibration.fingers.finger.proximal.jointstate = \
            CAL_BOUNDS_JOINT(desiredGlove->finger##Root1Raw, finger, proximal, desiredGlove->calibration.fingers); \
        state.uiState.currentCalibration.fingers.finger.proximal2.jointstate = \
            CAL_BOUNDS_JOINT(desiredGlove->finger##Root2Raw, finger, proximal2, desiredGlove->calibration.fingers); \
        state.uiState.currentCalibration.fingers.finger.distal.jointstate = \
            CAL_BOUNDS_JOINT(desiredGlove->finger##TipRaw, finger, distal, desiredGlove->calibration.fingers);
        #define COPY_THUMBBASE_STATE(jointstate) \
        state.uiState.currentCalibration.fingers.thumbBase.jointstate = \
            CAL_VALUE_BOUNDS(desiredGlove->thumbBaseRaw, desiredGlove->calibration.fingers.thumbBase);

        // Calibration code
        switch (state.uiState.calibrationState) {
            case CalibrationState_t::Fingers_DiscoverNeutral:
            {
                // Fingers are open, thumb is closed
                ImGui::Text("Rest your fingers naturally. Try aligning the fingers with the way you see them in VRChat for better accuracy.");

                FOREACH_FINGER(COPY_FINGER_STATE, rest)
                COPY_THUMBBASE_STATE(rest)

                // Override the fingers because we're mid-calibration
                desiredGlove->thumbRoot     = 0;
                desiredGlove->thumbTip      = 0;
                desiredGlove->indexRoot     = 0;
                desiredGlove->indexTip      = 0;
                desiredGlove->middleRoot    = 0;
                desiredGlove->middleTip     = 0;
                desiredGlove->ringRoot      = 0;
                desiredGlove->ringTip       = 0;
                desiredGlove->pinkyRoot     = 0;
                desiredGlove->pinkyTip      = 0;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = 0;
                desiredGlove->middleSplay = 0;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 0;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverClosed;
                }

                break;
            }
            case CalibrationState_t::Fingers_DiscoverClosed:
            {
                // Fingers are closed, thumb is expanded
                ImGui::Text("Close your hand. Try aligning the fingers with the way you see them in VRChat for better accuracy.");

                FOREACH_FINGER(COPY_FINGER_STATE, close)
                COPY_THUMBBASE_STATE(close)

                // Override the fingers because we're mid-calibration
                desiredGlove->thumbRoot     = 1;
                desiredGlove->thumbTip      = 1;
                desiredGlove->indexRoot     = 1;
                desiredGlove->indexTip      = 1;
                desiredGlove->middleRoot    = 1;
                desiredGlove->middleTip     = 1;
                desiredGlove->ringRoot      = 1;
                desiredGlove->ringTip       = 1;
                desiredGlove->pinkyRoot     = 1;
                desiredGlove->pinkyTip      = 1;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = 0;
                desiredGlove->middleSplay = 0;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 1;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverHorns;
                }

                break;
            }
            case CalibrationState_t::Fingers_DiscoverHorns:{
                ImGui::Text("horns");
                FOREACH_FINGER(COPY_FINGER_STATE, horns)
                COPY_THUMBBASE_STATE(horns)

                desiredGlove->thumbRoot     = 1;
                desiredGlove->thumbTip      = 0;
                desiredGlove->indexRoot     = 0;
                desiredGlove->indexTip      = 0;
                desiredGlove->middleRoot    = 1;
                desiredGlove->middleTip     = 1;
                desiredGlove->ringRoot      = 1;
                desiredGlove->ringTip       = 1;
                desiredGlove->pinkyRoot     = 0;
                desiredGlove->pinkyTip      = 0;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = 0;
                desiredGlove->middleSplay = 0;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 1;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverPeace;
                }
                break;
            }
            case CalibrationState_t::Fingers_DiscoverPeace:{
                ImGui::Text("peace");
                FOREACH_FINGER(COPY_FINGER_STATE, peace)
                COPY_THUMBBASE_STATE(peace)

                desiredGlove->thumbRoot     = 1;
                desiredGlove->thumbTip      = 0;
                desiredGlove->indexRoot     = 0;
                desiredGlove->indexTip      = 0;
                desiredGlove->middleRoot    = 0;
                desiredGlove->middleTip     = 0;
                desiredGlove->ringRoot      = 1;
                desiredGlove->ringTip       = 1;
                desiredGlove->pinkyRoot     = 1;
                desiredGlove->pinkyTip      = 1;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = -1;
                desiredGlove->middleSplay = 1;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 1;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverFlipoff;
                }
                break;
            }
            case CalibrationState_t::Fingers_DiscoverFlipoff:{
                ImGui::Text("flipoff");
                FOREACH_FINGER(COPY_FINGER_STATE, flipoff)
                COPY_THUMBBASE_STATE(flipoff)

                desiredGlove->thumbRoot     = 1;
                desiredGlove->thumbTip      = 1;
                desiredGlove->indexRoot     = 1;
                desiredGlove->indexTip      = 1;
                desiredGlove->middleRoot    = 0;
                desiredGlove->middleTip     = 0;
                desiredGlove->ringRoot      = 1;
                desiredGlove->ringTip       = 1;
                desiredGlove->pinkyRoot     = 1;
                desiredGlove->pinkyTip      = 1;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = 0;
                desiredGlove->middleSplay = 0;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 1;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverPoint;
                }
                break;
            }
            case CalibrationState_t::Fingers_DiscoverPoint:{
                ImGui::Text("point");
                FOREACH_FINGER(COPY_FINGER_STATE, point)
                COPY_THUMBBASE_STATE(point)

                desiredGlove->thumbRoot     = 1;
                desiredGlove->thumbTip      = 0;
                desiredGlove->indexRoot     = 0;
                desiredGlove->indexTip      = 0;
                desiredGlove->middleRoot    = 1;
                desiredGlove->middleTip     = 1;
                desiredGlove->ringRoot      = 1;
                desiredGlove->ringTip       = 1;
                desiredGlove->pinkyRoot     = 1;
                desiredGlove->pinkyTip      = 1;
                desiredGlove->thumbSplay = 0;
                desiredGlove->indexSplay = 0;
                desiredGlove->middleSplay = 0;
                desiredGlove->ringSplay = 0;
                desiredGlove->pinkySplay = 0;
                desiredGlove->thumbBase = 1;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::Fingers_MorePoses;
                    state.uiState.calibrationPoseIdx = 0;
                    calPoses->clear();
                }
                break;
            }
            case CalibrationState_t::Fingers_MorePoses:{
                if(state.uiState.calibrationPoseIdx >= CalPoseCount){
                    // empty frame, but prevents segfaults
                    state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverSplayed;
                    break;
                }
                ImGui::Text("Try to match the pose you can see in VR as close as possible.");
                ImGui::Text("%s", CalPoses[state.uiState.calibrationPoseIdx].text);

                #define COPY_SHOW_FINGER(finger, dummy) \
                    desiredGlove->finger##Root = CalPoses[state.uiState.calibrationPoseIdx].pose.finger.root; \
                    desiredGlove->finger##Tip = CalPoses[state.uiState.calibrationPoseIdx].pose.finger.tip; \
                    desiredGlove->finger##Splay = CalPoses[state.uiState.calibrationPoseIdx].pose.finger.splay;

                FOREACH_FINGER(COPY_SHOW_FINGER)
                desiredGlove->thumbBase = CalPoses[state.uiState.calibrationPoseIdx].pose.thumbBase;
                #undef COPY_SHOW_FINGER
                
                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    RecordedCalibrationPose recordedPose;

                    #define COPY_FINGER_STATE2(finger, dummy) \
                    recordedPose.sensors.finger.root1 = \
                    CAL_BOUNDS_JOINT(desiredGlove->finger##Root1Raw, finger, proximal, desiredGlove->calibration.fingers); \
                    recordedPose.sensors.finger.root2 = \
                    CAL_BOUNDS_JOINT(desiredGlove->finger##Root2Raw, finger, proximal2, desiredGlove->calibration.fingers); \
                    recordedPose.sensors.finger.tip = \
                    CAL_BOUNDS_JOINT(desiredGlove->finger##TipRaw, finger, distal, desiredGlove->calibration.fingers);

                    FOREACH_FINGER(COPY_FINGER_STATE2)
                    #undef COPY_FINGER_STATE2
                    recordedPose.sensors.thumbBase = CAL_VALUE_BOUNDS(desiredGlove->thumbBaseRaw, 
                        desiredGlove->calibration.fingers.thumbBase);
                    recordedPose.pose = CalPoses[state.uiState.calibrationPoseIdx].pose;

                    calPoses->emplace_back(recordedPose);

                    state.uiState.calibrationPoseIdx++;
                    if(state.uiState.calibrationPoseIdx >= CalPoseCount){
                        // Move to the next state
                        state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverSplayed;
                    }
                }

                break;
            }
            case CalibrationState_t::Fingers_DiscoverSplayed:
            {
                ImGui::Text("Spread your fingers apart, otherwise keeping them open (splay)");

                FOREACH_FINGER(COPY_FINGER_STATE, splayed)
                COPY_THUMBBASE_STATE(splayed)

                desiredGlove->thumbRoot     = 0;
                desiredGlove->thumbTip      = 0;
                desiredGlove->indexRoot     = 0;
                desiredGlove->indexTip      = 0;
                desiredGlove->middleRoot    = 0;
                desiredGlove->middleTip     = 0;
                desiredGlove->ringRoot      = 0;
                desiredGlove->ringTip       = 0;
                desiredGlove->pinkyRoot     = 0;
                desiredGlove->pinkyTip      = 0;
                desiredGlove->thumbSplay = 1;
                desiredGlove->indexSplay = 1;
                desiredGlove->middleSplay = 1;
                desiredGlove->ringSplay = 1;
                desiredGlove->pinkySplay = 1;
                desiredGlove->thumbBase = 0;

                // Proceed on any input
                ImGui::Text("Press any button to continue...");

                if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                    // Move to the next state
                    // state.uiState.calibrationState = CalibrationState::Fingers_DiscoverClosed;

                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    state.uiState.page = ScreenState_t::ScreenStateViewData;

                    // Copy the new calibration back
                    memcpy(&desiredGlove->calibration, &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
                    solver->calibrate(desiredGlove->calibration.fingers, *calPoses);
                }
                break;
            }
            case CalibrationState_t::Fingers_DiscoverBackwardsBend:
            {
                // Maybe unecessary, with linear interpolation this should be a given

                break;
            }
            default:
            {
                ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
                break;
            }
        }

        if (ImGui::Button("Cancel")) {
            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;

            // Set the old calibration
            // Should not be necessary

        }
    }
    ImGui::EndGroupPanel();
}

void DrawCalibrateFingersSingle(AppState& state) {

    ImGui::BeginGroupPanel("Single Finger Calibration");
    {
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Glove Finger...");
            desiredGlove = &state.gloveLeft;
        }
        else {
            ImGui::Text("Calibrating Right Glove Finger...");
            desiredGlove = &state.gloveRight;
        }

        // Handled here so that we don't get a blank frame
        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));

            // Create a new calibration state, and 0 out the finger calibration
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            memset(&state.uiState.currentCalibration.fingers, 0, sizeof(protocol::ContactGloveState_t::FingerCalibrationData_t));

            // Done. Move to DiscoverBounds
            state.uiState.calibrationState = CalibrationState_t::Fingers_FocusOnFinger;
        }

        if (state.uiState.calibrationState == CalibrationState_t::Fingers_FocusOnFinger) {
            ImGui::Text("Pick a finger to calibrate:");

            if (ImGui::Button("Thumb")) {
                state.uiState.targetFinger = CalibrationFinger_t::Finger_Thumb;
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
            }
            if (ImGui::Button("Index")) {
                state.uiState.targetFinger = CalibrationFinger_t::Finger_Index;
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
            }
            if (ImGui::Button("Middle")) {
                state.uiState.targetFinger = CalibrationFinger_t::Finger_Middle;
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
            }
            if (ImGui::Button("Ring")) {
                state.uiState.targetFinger = CalibrationFinger_t::Finger_Ring;
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
            }
            if (ImGui::Button("Pinky")) {
                state.uiState.targetFinger = CalibrationFinger_t::Finger_Pinky;
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverNeutral;
            }
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown || state.uiState.gloveButtons.releasedLeft.joystickClick ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown || state.uiState.gloveButtons.releasedRight.joystickClick;

        // Calibration code
        switch (state.uiState.calibrationState) {
        case CalibrationState_t::Fingers_DiscoverNeutral:
        {
            // Fingers are open, thumb is closed
            ImGui::Text("Rest your fingers naturally, and close your thumb. Try aligning the fingers with the way you see them in VRChat for better accuracy.");

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Thumb) {
                state.uiState.currentCalibration.fingers.thumb.proximal.close   = desiredGlove->thumbRoot1Raw;
                state.uiState.currentCalibration.fingers.thumb.distal.close     = desiredGlove->thumbTipRaw;
                desiredGlove->thumbRoot = 1;
                desiredGlove->thumbTip = 1;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Index) {
                state.uiState.currentCalibration.fingers.index.proximal.rest    = desiredGlove->indexRoot1Raw;
                state.uiState.currentCalibration.fingers.index.distal.rest      = desiredGlove->indexTipRaw;
                desiredGlove->indexRoot = 0;
                desiredGlove->indexTip = 0;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Middle) {
                state.uiState.currentCalibration.fingers.middle.proximal.rest   = desiredGlove->middleRoot1Raw;
                state.uiState.currentCalibration.fingers.middle.distal.rest     = desiredGlove->middleTipRaw;
                desiredGlove->middleRoot = 0;
                desiredGlove->middleTip = 0;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Ring) {
                state.uiState.currentCalibration.fingers.ring.proximal.rest     = desiredGlove->ringRoot1Raw;
                state.uiState.currentCalibration.fingers.ring.distal.rest       = desiredGlove->ringTipRaw;
                desiredGlove->ringRoot = 0;
                desiredGlove->ringTip = 0;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Pinky) {
                state.uiState.currentCalibration.fingers.pinky.proximal.rest    = desiredGlove->pinkyRoot1Raw;
                state.uiState.currentCalibration.fingers.pinky.distal.rest      = desiredGlove->pinkyTipRaw;
                desiredGlove->pinkyRoot = 0;
                desiredGlove->pinkyTip = 0;
            }

            // Proceed on any input
            ImGui::Text("Press any button to continue...");

            if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                // Move to the next state
                state.uiState.calibrationState = CalibrationState_t::Fingers_DiscoverClosed;
            }

            break;
        }
        case CalibrationState_t::Fingers_DiscoverClosed:
        {
            // Fingers are closed, thumb is expanded
            ImGui::Text("Close your fingers, and rest your thumb naturally. Try aligning the fingers with the way you see them in VRChat for better accuracy.");

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Thumb) {
                state.uiState.currentCalibration.fingers.thumb.proximal.rest    = desiredGlove->thumbRoot1Raw;
                state.uiState.currentCalibration.fingers.thumb.distal.rest      = desiredGlove->thumbTipRaw;
                desiredGlove->thumbRoot = 0;
                desiredGlove->thumbTip = 0;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Index) {
                state.uiState.currentCalibration.fingers.index.proximal.close   = desiredGlove->indexRoot1Raw;
                state.uiState.currentCalibration.fingers.index.distal.close     = desiredGlove->indexTipRaw;
                desiredGlove->indexRoot = 1;
                desiredGlove->indexTip = 1;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Middle) {
                state.uiState.currentCalibration.fingers.middle.proximal.close  = desiredGlove->middleRoot1Raw;
                state.uiState.currentCalibration.fingers.middle.distal.close    = desiredGlove->middleTipRaw;
                desiredGlove->middleRoot = 1;
                desiredGlove->middleTip = 1;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Ring) {
                state.uiState.currentCalibration.fingers.ring.proximal.close    = desiredGlove->ringRoot1Raw;
                state.uiState.currentCalibration.fingers.ring.distal.close      = desiredGlove->ringTipRaw;
                desiredGlove->ringRoot = 1;
                desiredGlove->ringTip = 1;
            }

            if (state.uiState.targetFinger == CalibrationFinger_t::Finger_Pinky) {
                state.uiState.currentCalibration.fingers.pinky.proximal.close   = desiredGlove->pinkyRoot1Raw;
                state.uiState.currentCalibration.fingers.pinky.distal.close     = desiredGlove->pinkyTipRaw;
                desiredGlove->pinkyRoot = 1;
                desiredGlove->pinkyTip = 1;
            }

            // Proceed on any input
            ImGui::Text("Press any button to continue...");

            if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                // Move to the next state
                // state.uiState.calibrationState = CalibrationState::Fingers_DiscoverClosed;

                // Move to the next state
                state.uiState.calibrationState = CalibrationState_t::State_Entering;
                state.uiState.page = ScreenState_t::ScreenStateViewData;

                // Copy the new calibration back
                memcpy(&desiredGlove->calibration, &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            }

            break;
        }
        case CalibrationState_t::Fingers_DiscoverBackwardsBend:
        {
            // Maybe unecessary, with linear interpolation this should be a given

            break;
        }
        default:
        {
            ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
            break;
        }
        }

        if (ImGui::Button("Cancel")) {
            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;

            // Set the old calibration
            // Should not be necessary

        }
    }
    ImGui::EndGroupPanel();
}


void DrawCalibrateOffsets(AppState& state) {
#ifdef WIN32
    ImGui::BeginGroupPanel("Pose Offset Calibration");
    {
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Glove Pose Offset...");
            desiredGlove = &state.gloveLeft;
        } else {
            ImGui::Text("Calibrating Right Glove Pose Offset...");
            desiredGlove = &state.gloveRight;
        }

        // Handled here so that we don't get a blank frame
        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));

            // Create a new calibration state, and 0 out the pose calibration
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            memset(&state.uiState.currentCalibration.poseOffset, 0, sizeof(protocol::ContactGloveState_t::CalibrationData_t::PoseOffset_t));

            // Done. Move to DiscoverBounds
            state.uiState.calibrationState = CalibrationState_t::PoseOffset_FindInitialPose;
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown || state.uiState.gloveButtons.releasedLeft.joystickClick ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown || state.uiState.gloveButtons.releasedRight.joystickClick;

        // Calibration code
        switch (state.uiState.calibrationState) {
            case CalibrationState_t::PoseOffset_FindInitialPose:
            {
                ImGui::Text("Waiting for a valid pose. Please make sure your tracker is on, and tracking.");
                
                // Request a driver pose from the server
                protocol::Request_t req = protocol::Request_t(desiredGlove->trackerIndex);
                protocol::Response_t response = state.ipcClient->SendBlocking(req);

                if (response.type == protocol::ResponseDevicePose) {

                    vr::DriverPose_t driverPose = response.driverPose;

                    const vr::HmdVector3d_t driverPosition = { driverPose.vecPosition[0], driverPose.vecPosition[1], driverPose.vecPosition[2] };
                    state.uiState.initialTrackerPos = driverPosition + (desiredGlove->calibration.poseOffset.pos * driverPose.qRotation);
                    state.uiState.initialTrackerRot = driverPose.qRotation * desiredGlove->calibration.poseOffset.rot;

                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::PoseOffset_Moving;

                    // Tell the driver to ignore pose updates, we shall monitor the device's transform in the meantime
                    desiredGlove->ignorePose = true;
                }

                break;
            }
        case CalibrationState_t::PoseOffset_Moving:
        {
            ImGui::Text("Line up the glove with your hand. Once you find the right spot, press any button to assign the offset.");

            // Tell the driver to ignore pose updates, we shall monitor the device's transform in the meantime
            desiredGlove->ignorePose = true;

            // Proceed on any input
            ImGui::Text("Press any button to continue...");

            if (ImGui::Button("Continue") || anyButtonPressedJoystick) {
                #ifdef WIN32
                // Request a driver pose from the server
                protocol::Request_t req = protocol::Request_t(desiredGlove->trackerIndex);
                protocol::Response_t response = state.ipcClient->SendBlocking(req);

                if (response.type == protocol::ResponseDevicePose) {

                    vr::DriverPose_t driverPose = response.driverPose;

                    const vr::HmdVector3d_t driverPosition = { driverPose.vecPosition[0], driverPose.vecPosition[1], driverPose.vecPosition[2] };
                    const vr::HmdVector3d_t newPos = driverPosition + (desiredGlove->calibration.poseOffset.pos * driverPose.qRotation);
                    const vr::HmdQuaternion_t newRot = driverPose.qRotation * desiredGlove->calibration.poseOffset.rot;

                    // Compute the deltas...
                    // Maths taken from openglove driver
                    const vr::HmdQuaternion_t transformQuat = -newRot * state.uiState.initialTrackerRot;

                    const vr::HmdVector3d_t differenceVector = state.uiState.initialTrackerPos - newPos;
                    const vr::HmdVector3d_t transformVector = differenceVector * -newRot;

                    state.uiState.currentCalibration.poseOffset.rot = transformQuat;
                    state.uiState.currentCalibration.poseOffset.pos = transformVector;

                    // Tell the driver to stop ignoring pose updates, we have an offset now!
                    desiredGlove->ignorePose = false;

                    // Copy the new calibration back
                    memcpy(&desiredGlove->calibration, &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));

                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    state.uiState.page = ScreenState_t::ScreenStateViewData;
                }
                #endif
            }

            break;
        }
            // Maybe unecessary, with linear interpolation this should be a given
        default:
        {
            ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
            break;
        }
        }

        if (ImGui::Button("Cancel")) {
            // Tell the driver to stop ignoring pose updates, we have decided to stop calibrating the offset
            desiredGlove->ignorePose = false;

            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;
        }
    }
    ImGui::EndGroupPanel();
#else
    state.uiState.calibrationState = CalibrationState_t::State_Entering;
    state.uiState.page = ScreenState_t::ScreenStateViewData;
#endif

}

void DrawCalibrateSensorBounds(AppState& state){
    ImGui::BeginGroupPanel("Sensor Bounds Calibration");
    {
    // Isolate the glove we wish to work on
        protocol::ContactGloveState_t* desiredGlove = nullptr;
        if (state.uiState.processingHandedness == Handedness_t::Left) {
            ImGui::Text("Calibrating Left Sensors...");
            desiredGlove = &state.gloveLeft;
        }
        else {
            ImGui::Text("Calibrating Right Sensors...");
            desiredGlove = &state.gloveRight;
        }

        if (state.uiState.calibrationState == CalibrationState_t::State_Entering) {
            // Copy the old calibration data
            memcpy(&state.uiState.oldCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
            // Create a new calibration state
            memcpy(&state.uiState.currentCalibration, &desiredGlove->calibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));

            #define ZERO_FINGER_BOUNDS(finger, joint) \
            state.uiState.currentCalibration.fingers.finger.joint.min = 65535; \
            state.uiState.currentCalibration.fingers.finger.joint.max = 0;

            FOREACH_FINGER(ZERO_FINGER_BOUNDS, proximal)
            FOREACH_FINGER(ZERO_FINGER_BOUNDS, proximal2)
            FOREACH_FINGER(ZERO_FINGER_BOUNDS, distal)
            state.uiState.currentCalibration.fingers.thumbBase.min = 65535;
            state.uiState.currentCalibration.fingers.thumbBase.max = 0;
            #undef ZERO_FINGER_BOUNDS
            state.uiState.calibrationState = CalibrationState_t::Bounds_Measure;
        }

        // Cache if the buttons are pressed
        const bool anyButtonPressedNoJoystick =
            state.uiState.gloveButtons.releasedLeft.buttonUp || state.uiState.gloveButtons.releasedLeft.buttonDown || state.uiState.gloveButtons.releasedLeft.systemUp || state.uiState.gloveButtons.releasedLeft.systemDown ||
            state.uiState.gloveButtons.releasedRight.buttonUp || state.uiState.gloveButtons.releasedRight.buttonDown || state.uiState.gloveButtons.releasedRight.systemUp || state.uiState.gloveButtons.releasedRight.systemDown;

        switch (state.uiState.calibrationState) {
            case CalibrationState_t::Bounds_Measure:
            {
                ImGui::Text("Make different poses with your hand, moving all joints to their extremes");
                 
                #define UPDATE_FINGER_BOUNDS(finger, joint, joint2) \
                state.uiState.currentCalibration.fingers.finger.joint.min = \
                    MIN(state.uiState.currentCalibration.fingers.finger.joint.min, desiredGlove->finger##joint2##Raw); \
                state.uiState.currentCalibration.fingers.finger.joint.max = \
                    MAX(state.uiState.currentCalibration.fingers.finger.joint.max, desiredGlove->finger##joint2##Raw);

                FOREACH_FINGER(UPDATE_FINGER_BOUNDS, proximal, Root1)
                FOREACH_FINGER(UPDATE_FINGER_BOUNDS, proximal2, Root2)
                FOREACH_FINGER(UPDATE_FINGER_BOUNDS, distal, Tip)
                state.uiState.currentCalibration.fingers.thumbBase.min = MIN(
                    state.uiState.currentCalibration.fingers.thumbBase.min, desiredGlove->thumbBaseRaw);
                state.uiState.currentCalibration.fingers.thumbBase.max = MAX(
                    state.uiState.currentCalibration.fingers.thumbBase.max, desiredGlove->thumbBaseRaw);

                // Proceed on any input
                ImGui::Text("Press any button to continue... (or click the trigger)");

                if (ImGui::Button("Continue") || anyButtonPressedNoJoystick) {
                    // Move to the next state
                    state.uiState.calibrationState = CalibrationState_t::State_Entering;
                    state.uiState.page = ScreenState_t::ScreenStateViewData;

                    // Copy the new calibration back
                    memcpy(&desiredGlove->calibration , &state.uiState.currentCalibration, sizeof(protocol::ContactGloveState_t::CalibrationData_t));
                }
                break;
            }
            default:
            {
                ImGui::Text((std::string("Encountered broken state! (") + std::to_string((int)state.uiState.calibrationState) + ")").c_str());
                break;
            }
        }    
        if (ImGui::Button("Cancel")) {
            // Move to the data page
            state.uiState.calibrationState = CalibrationState_t::State_Entering;
            state.uiState.page = ScreenState_t::ScreenStateViewData;

            // Set the old calibration
            // Should not be necessary
            // memcpy(&desiredGlove->calibration, &state.uiState.oldCalibration, sizeof(protocol::ContactGloveState::CalibrationData));
        }        
    }
    ImGui::EndGroupPanel();
}

void DrawUi(const bool isOverlay, AppState& state) {
#ifdef _DEBUG
    ImGui::ShowDemoWindow();
#endif

    const ImGuiIO& io = ImGui::GetIO();

    // Lock window to full screen
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

    ImGui::Begin("FreeScuba - Open source contact gloves driver", 0,
        // Since this is locked to the window, we want to disable:
        //   the titlebar,
        //   resizing and moving
        //   collapsing the window by double clicking
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse
    );

    switch (state.uiState.page) {
        case ScreenState_t::ScreenStateViewData: {
            ImGui::Checkbox("Use neural net for glove processing", &state.useNN);
            if (state.uiState.page == ScreenState_t::ScreenStateViewData) {
                state.uiState.processingHandedness = Handedness_t::Left;
            }
            DrawGlove("Left Glove", "glove_left", state.gloveLeft, state);
            if (state.uiState.page == ScreenState_t::ScreenStateViewData) {
                state.uiState.processingHandedness = Handedness_t::Right;
            }
            DrawGlove("Right Glove", "glove_right", state.gloveRight, state);

            // @TODO: Break settings into function / tab
            ImGui::Spacing();
            ImGui::Checkbox("Automatically launch with SteamVR", &state.doAutoLaunch);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateJoystick: {
            DrawCalibrateJoystick(state);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateFingers: {
            DrawCalibrateFingers(state);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateFingersSingle: {
            DrawCalibrateFingersSingle(state);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateOffset: {
            DrawCalibrateOffsets(state);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateTrigger: {
            DrawCalibrateTrigger(state);
            break;
        }
        case ScreenState_t::ScreenStateCalibrateSensorBounds: {
            DrawCalibrateSensorBounds(state);
            break;
        }
    }

    ImGui::End();
}