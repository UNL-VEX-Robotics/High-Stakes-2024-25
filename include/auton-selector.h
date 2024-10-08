#pragma once
#include "vex.h"

class button{
private:
    int x;
    int y;
    int width;
    int height;

    vex::color backgroundColor;
    vex::color outlineColor;
    int outlineThickness;
    vex::color textColor;

    const char* text;

    vex::brain::lcd* Screen;
public:
    button();
    button(vex::brain::lcd* Screen, int x, int y, int width, int height, vex::color backgroundColor, vex::color outlineColor, int outlineThickness, vex::color textColor, const char* text);
    void draw();
    void changeColor(vex::color backgroundColor, vex::color outlineColor, vex::color textColor);
    bool isPressing();
};

class selector{
public:
    enum buttonType{
        page,
        color,
        route,
        __break
    };
private:
    std::vector<const char*> pages = {"none"};

    vex::brain::lcd* Screen;

    struct {
        vex::color background;
    }main_screen;

    struct page_buttons{
        vex::color outlineColor;
        vex::color selectedOutlineColor;
        vex::color backgroundColor;
        vex::color selectedBackgroundColor;
        vex::color textColor;
        vex::color selectedTextColor;

        bool isSelected;

        int height;
        int outlineThickness;
        int pageNum;
    };

    struct auton_buttons{
        vex::color outlineColor;
        vex::color selectedOutlineColor;
        vex::color backgroundColor;
        vex::color selectedBackgroundColor;
        vex::color textColor;
        vex::color selectedTextColor;

        bool isSelected;

        int width;
        int height;
        int outlineThickness;
    };

    struct color_buttons{
        vex::color defaultOutlineColor;
        vex::color secondaryOutlineColor;
        vex::color defaultColor;
        vex::color secondaryColor;
        vex::color defaultTextColor;
        vex::color secondaryTextColor;

        bool isRed;

        int width;
        int height;
        int outlineThickness;
    };

    page_buttons page_defaults;
    auton_buttons auton_defaults;
    color_buttons color_defaults;

    bool pageIsColor;

    struct button_data{
        button_data();

        buttonType type;
        const char* page;
        const char* buttonName;

        page_buttons page_buttons_data;
        auton_buttons auton_buttons_data;
        color_buttons color_buttons_data;

        button Button;
    };

    std::vector<button_data> buttons;

public:
    selector(vex::brain::lcd* Screen, bool usePagesForColor = false);

    void setScreenColor(vex::color background);
    void setPageDefaults(vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor, int height, int outlineThickness);
    void setAutonDefaults(vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor, int width, int height, int outlineThickness);
    void setColorDefaults(vex::color defaultOutlineColor, vex::color defaultBackgroundColor, vex::color defaultTextColor, vex::color secondaryOutlineColor, vex::color secondaryBackgroundColor, vex::color secondaryTextColor, int width, int height, int outlineThickness, bool redDefault);

    void addPage(const char* text);
    void addPage(const char* text, vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor);

    void addAuton(int x, int y, const char* text, const char* page, bool usePageColors = false);
    void addAuton(int x, int y, const char* text, const char* page, vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor);

    void addColor(int x, int y, const char* text, const char* page);
    void addColor(int x, int y, const char* text, const char* page, vex::color defaultOutlineColor, vex::color defaultBackgroundColor, vex::color defaultTextColor, vex::color secondaryOutlineColor, vex::color secondaryBackgroundColor, vex::color secondaryTextColor);

    void addBreak(int x, int y, const char* text, const char* page, vex::color outlineColor, vex::color backgroundColor, vex::color textColor);

    std::vector<const char*> runSelection();
};