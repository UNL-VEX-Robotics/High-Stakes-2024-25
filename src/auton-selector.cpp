#include "auton-selector.h"

/**
 * default constructor for button class
 * dumb solution to a dumb problem
 */
button::button(){}
/**
 * Constructor method that creates a button
 * 
 * @param   Screen              a pointer to the brain's screen
 * @param   x                   the x coordinate of the upper-left corner
 * @param   y                   the y coordinate of the upper-left corner
 * @param   width               the width of the button
 * @param   height              the height of the button
 * @param   backgroundColor     the background color of the button
 * @param   outlineColor        the outline color of the button
 * @param   outlineThickness    the thickness of the outline
 * @param   textColor           the color of the text
 * @param   text                the text on the button
 */
button::button(vex::brain::lcd* Screen, int x, int y, int width, int height, vex::color backgroundColor, vex::color outlineColor, int outlineThickness, vex::color textColor, const char* text)
{
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
    this->backgroundColor = backgroundColor;
    this->outlineColor = outlineColor;
    this->outlineThickness = outlineThickness;
    this->textColor = textColor;
    this->text = text;
    this->Screen = Screen;
}

/**
 * Draws the button on the screen
 */
void button::draw()
{
    this->Screen->setPenColor(this->outlineColor);
    this->Screen->setFillColor(this->backgroundColor);
    this->Screen->setPenWidth(outlineThickness);
    this->Screen->drawRectangle(this->x, this->y, this->width, this->height);
    int textWidth = Screen->getStringWidth(this->text);
    int textHeight = Screen->getStringHeight(this->text);

    Screen->setPenColor(this->textColor);
    Screen->printAt(this->x + (this->width / 2) -  (textWidth / 2), this->y + (this->height / 2) + (textHeight / 2), this->text);
}

/**
 * Changes the button's colors
 * 
 * @param   backgroundColor     the background color of the button
 * @param   outlineColor        the outline color of the button
 * @param   textColor           the color of the text
 */
void button::changeColor(vex::color backgroundColor, vex::color outlineColor, vex::color textColor)
{
    this->backgroundColor = backgroundColor;
    this->outlineColor = outlineColor;
    this->textColor = textColor;
}

/**
 * Checks if the button is being pressed
 * 
 * @return  true if being pressed, false otherwise
 */
bool button::isPressing()
{
    return (Screen->xPosition() > this->x && Screen->xPosition() < this->x + this->width && this->Screen->yPosition() > this->y && this->Screen->yPosition() < this->y + this->height);
}

/**
 * Constructor method for the selector class
 * 
 * @param   Screen  a pointer to the brain's screen
 */
selector::selector(vex::brain::lcd *Screen, bool usePagesForColor)
{
    this->Screen = Screen;
    this->pageIsColor = usePagesForColor;
}

/**
 * Sets the main background color
 * 
 * @param   background  the backgroundColor
 */
void selector::setScreenColor(vex::color background)
{
    this->main_screen.background = background;
}

/**
 * Sets the defaults for page buttons
 * 
 * @param   outlineColor            the base outline color of the button
 * @param   backgroundColor         the base background color of the button
 * @param   textColor               the base text color of the button
 * @param   selectedOutlineColor    the color of the outline while the button is selected
 * @param   selectedBackgroundColor the color of the background while the button is selected
 * @param   selectedTextColor       the color of the text while the button is selected
 * @param   height                  the height of the button
 * @param   outlineThickness        the thickness of the outline of the button
 */
void selector::setPageDefaults(vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor, int height, int outlineThickness)
{
    this->page_defaults.outlineColor = outlineColor;
    this->page_defaults.backgroundColor = backgroundColor;
    this->page_defaults.textColor = textColor;
    this->page_defaults.selectedOutlineColor = selectedOutlineColor;
    this->page_defaults.selectedBackgroundColor = selectedBackgroundColor;
    this->page_defaults.selectedTextColor = selectedTextColor;
    this->page_defaults.height = height;
    this->page_defaults.outlineThickness = outlineThickness;
}

/**
 * Sets the defaults for auton buttons
 * 
 * @param   outlineColor            the base outline color of the button
 * @param   backgroundColor         the base background color of the button
 * @param   textColor               the base text color of the button
 * @param   selectedOutlineColor    the color of the outline while the button is selected
 * @param   selectedBackgroundColor the color of the background while the button is selected
 * @param   selectedTextColor       the color of the text while the button is selected
 * @param   width                   the width of the button
 * @param   height                  the height of the button
 * @param   outlineThickness        the thickness of the outline of the button
 */
void selector::setAutonDefaults(vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor, int width, int height,int outlineThickness)
{
    this->auton_defaults.outlineColor = outlineColor;
    this->auton_defaults.backgroundColor = backgroundColor;
    this->auton_defaults.textColor = textColor;
    this->auton_defaults.selectedOutlineColor = selectedOutlineColor;
    this->auton_defaults.selectedBackgroundColor = selectedBackgroundColor;
    this->auton_defaults.selectedTextColor = selectedTextColor;
    this->auton_defaults.width = width;
    this->auton_defaults.height = height;
    this->auton_defaults.outlineThickness = outlineThickness;
}

/**
 * Sets the defaults for auton buttons
 * 
 * @param   defaultOutlineColor         the outline color for the default color
 * @param   defaultBackgroundColor      the background color for the default color
 * @param   defaultTextColor            the text color for the default color
 * @param   secondaryOutlineColor       the outline color for the secondary color
 * @param   secondaryBackgroundColor    the background color for the secondary color
 * @param   secondaryTextColor          the text color for the secondary color
 * @param   width                       the width of the button
 * @param   height                      the height of the button
 * @param   outlineThickness            the thickness of the outline of the button
 */
void selector::setColorDefaults(vex::color defaultOutlineColor, vex::color defaultBackgroundColor, vex::color defaultTextColor, vex::color secondaryOutlineColor, vex::color secondaryBackgroundColor, vex::color secondaryTextColor, int width, int height, int outlineThickness, bool redDefault)
{
    this->color_defaults.defaultOutlineColor = defaultOutlineColor;
    this->color_defaults.defaultColor = defaultBackgroundColor;
    this->color_defaults.defaultTextColor = defaultTextColor;
    this->color_defaults.secondaryOutlineColor = secondaryOutlineColor;
    this->color_defaults.secondaryColor = secondaryBackgroundColor;
    this->color_defaults.secondaryTextColor = secondaryTextColor;
    this->color_defaults.width = width;
    this->color_defaults.height = height;
    this->color_defaults.outlineThickness = outlineThickness;
    this->color_defaults.isRed = redDefault;
}

/**
 * Adds a page to the screen
 * 
 * @param   text    the text to be displayed
 */
void selector::addPage(const char *text)
{
    this->addPage(text, this->page_defaults.outlineColor, this->page_defaults.backgroundColor, this->page_defaults.textColor, this->page_defaults.selectedOutlineColor, this->page_defaults.selectedBackgroundColor, this->page_defaults.selectedTextColor);
}

/**
 * Adds a page to the screen
 * 
 * @param   text                    the text to be displayed
 * @param   outlineColor            the base outline color of the button
 * @param   backgroundColor         the base background color of the button
 * @param   textColor               the base text color of the button
 * @param   selectedOutlineColor    the color of the outline while the button is selected
 * @param   selectedBackgroundColor the color of the background while the button is selected
 * @param   selectedTextColor       the color of the text while the button is selected
 */
void selector::addPage(const char *text, vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor)
{
    button_data this_button_data;

    this_button_data.type = buttonType::page;
    this_button_data.page = "none";
    this_button_data.buttonName = text;

    this_button_data.page_buttons_data.outlineColor = outlineColor;
    this_button_data.page_buttons_data.backgroundColor = backgroundColor;
    this_button_data.page_buttons_data.textColor = textColor;
    this_button_data.page_buttons_data.selectedOutlineColor = selectedOutlineColor;
    this_button_data.page_buttons_data.selectedBackgroundColor = selectedBackgroundColor;
    this_button_data.page_buttons_data.selectedTextColor = selectedTextColor;
    this_button_data.page_buttons_data.isSelected = false;
    this_button_data.page_buttons_data.height = this->page_defaults.height;
    this_button_data.page_buttons_data.outlineThickness = this->page_defaults.outlineThickness;

    bool uniquePage = true;
    for (int i = 0; i < this->pages.size(); i++){
        if(text == this->pages.at(i)) uniquePage = false;
    }

    if(uniquePage) this->pages.push_back(text);

    this_button_data.page_buttons_data.pageNum = this->pages.size() - 2;

    this->buttons.push_back(this_button_data);
}

/**
 * Adds an auton button to the screen
 * 
 * @param   x                       the upper-left x coordinate
 * @param   y                       the upper-left y coordinate
 * @param   text            the text to be displayed
 * @param   page            what page the button falls under
 * @param   usePageColors   uses the page default colors rather than auton default colors
 */
void selector::addAuton(int x, int y, const char *text, const char *page, bool usePageColors)
{
    if(!usePageColors){
        this->addAuton(x, y, text, page, this->auton_defaults.outlineColor, this->auton_defaults.backgroundColor, this->auton_defaults.textColor, this->auton_defaults.selectedOutlineColor, this->auton_defaults.selectedBackgroundColor, this->auton_defaults.selectedTextColor);
    }
    else{
        this->addAuton(x, y, text, page, this->page_defaults.outlineColor, this->page_defaults.backgroundColor, this->page_defaults.textColor, this->page_defaults.selectedOutlineColor, this->page_defaults.selectedBackgroundColor, this->page_defaults.selectedTextColor);
    }
}

/**
 * Adds an auton button to the screen
 * 
 * @param   x                       the upper-left x coordinate
 * @param   y                       the upper-left y coordinate                     
 * @param   text                    the text to be displayed
 * @param   page                    what page the button falls under
 * @param   outlineColor            the base outline color of the button
 * @param   backgroundColor         the base background color of the button
 * @param   textColor               the base text color of the button
 * @param   selectedOutlineColor    the color of the outline while the button is selected
 * @param   selectedBackgroundColor the color of the background while the button is selected
 * @param   selectedTextColor       the color of the text while the button is selected
 */
void selector::addAuton(int x, int y, const char *text, const char *page, vex::color outlineColor, vex::color backgroundColor, vex::color textColor, vex::color selectedOutlineColor, vex::color selectedBackgroundColor, vex::color selectedTextColor)
{
    button_data this_button_data;

    this_button_data.type = buttonType::route;
    this_button_data.page = page;
    this_button_data.buttonName = text;

    this_button_data.auton_buttons_data.outlineColor = outlineColor;
    this_button_data.auton_buttons_data.backgroundColor = backgroundColor;
    this_button_data.auton_buttons_data.textColor = textColor;
    this_button_data.auton_buttons_data.selectedOutlineColor = selectedOutlineColor;
    this_button_data.auton_buttons_data.selectedBackgroundColor = selectedBackgroundColor;
    this_button_data.auton_buttons_data.selectedTextColor = selectedTextColor;
    this_button_data.auton_buttons_data.isSelected = false;
    this_button_data.auton_buttons_data.width = this->auton_defaults.width;
    this_button_data.auton_buttons_data.height = this->auton_defaults.height;
    this_button_data.auton_buttons_data.outlineThickness = this->auton_defaults.outlineThickness;

    this_button_data.Button = button(this->Screen, x, y, this_button_data.auton_buttons_data.width, this_button_data.auton_buttons_data.height, backgroundColor, outlineColor, this_button_data.auton_buttons_data.outlineThickness, textColor, text);

    this->buttons.push_back(this_button_data);
}

/**
 * Adds a color button to the screen
 * 
 * @param   x                       the upper-left x coordinate
 * @param   y                       the upper-left y coordinate
 * @param   text                    the text to be displayed
 * @param   page                    what page the button falls under
 */
void selector::addColor(int x, int y, const char *text, const char *page)
{
    this->addColor(x, y, text, page, this->color_defaults.defaultOutlineColor, this->color_defaults.defaultColor, this->color_defaults.defaultTextColor, this->color_defaults.secondaryOutlineColor, this->color_defaults.secondaryColor, this->color_defaults.secondaryTextColor);
}

/**
 * Adds a color button to the screen
 * 
 * @param   x                           the upper-left x coordinate
 * @param   y                           the upper-left y coordinate
 * @param   text                        the text to be displayed
 * @param   page                        what page the button falls under
 * @param   defaultOutlineColor         the base outline color of the button
 * @param   defaultBackgroundColor      the base background color of the button
 * @param   defaultTextColor            the base text color of the button
 * @param   secondaryOutlineColor       the color of the outline while the button is selected
 * @param   secondaryBackgroundColor    the color of the background while the button is selected
 * @param   secondaryTextColor          the color of the text while the button is selected
 */
void selector::addColor(int x, int y, const char *text, const char *page, vex::color defaultOutlineColor, vex::color defaultBackgroundColor, vex::color defaultTextColor, vex::color secondaryOutlineColor, vex::color secondaryBackgroundColor, vex::color secondaryTextColor)
{
    button_data this_button_data;

    this_button_data.type = buttonType::color;
    this_button_data.page = page;
    this_button_data.buttonName = text;

    this_button_data.color_buttons_data.defaultOutlineColor = defaultOutlineColor;
    this_button_data.color_buttons_data.defaultColor = defaultBackgroundColor;
    this_button_data.color_buttons_data.defaultTextColor = defaultTextColor;
    this_button_data.color_buttons_data.secondaryOutlineColor = secondaryOutlineColor;
    this_button_data.color_buttons_data.secondaryColor = secondaryBackgroundColor;
    this_button_data.color_buttons_data.secondaryTextColor = secondaryTextColor;
    this_button_data.color_buttons_data.isRed = this->color_defaults.isRed;
    this_button_data.color_buttons_data.width = this->color_defaults.width;
    this_button_data.color_buttons_data.height = this->color_defaults.height;
    this_button_data.color_buttons_data.outlineThickness = this->color_defaults.outlineThickness;

    this_button_data.Button = button(this->Screen, x, y, this_button_data.color_buttons_data.width, this_button_data.color_buttons_data.height, defaultBackgroundColor, defaultOutlineColor, this_button_data.color_buttons_data.outlineThickness, defaultTextColor, text);

    this->buttons.push_back(this_button_data);
}

/**
 * Adds a break button to the screen
 * 
 * @param   x                       the upper-left x coordinate
 * @param   y                       the upper-left y coordinate
 * @param   text                    the text to be displayed
 * @param   page                    what page the button falls under
 * @param   outlineColor            the base outline color of the button
 * @param   backgroundColor         the base background color of the button
 * @param   textColor               the base text color of the button
 */
void selector::addBreak(int x, int y, const char *text, const char *page, vex::color outlineColor, vex::color backgroundColor, vex::color textColor)
{
    button_data this_button_data;

    this_button_data.type = buttonType::__break;
    this_button_data.page = page;
    this_button_data.buttonName = text;

    this_button_data.auton_buttons_data.outlineColor = outlineColor;
    this_button_data.auton_buttons_data.backgroundColor = backgroundColor;
    this_button_data.auton_buttons_data.textColor = textColor;
    this_button_data.auton_buttons_data.isSelected = false;
    this_button_data.auton_buttons_data.width = this->auton_defaults.width;
    this_button_data.auton_buttons_data.height = this->auton_defaults.height;
    this_button_data.auton_buttons_data.outlineThickness = this->auton_defaults.outlineThickness;

    this_button_data.Button = button(this->Screen, x, y, this_button_data.auton_buttons_data.width, this_button_data.auton_buttons_data.height, backgroundColor, outlineColor, this_button_data.auton_buttons_data.outlineThickness, textColor, text);

    this->buttons.push_back(this_button_data);
}

/**
 * runs the auton selection
 * 
 * @return if page is color (page, auton) else (color, auton)
 */
std::vector<const char *> selector::runSelection()
{   
    bool isRed = this->color_defaults.isRed; //only used if there is color buttons

    //generate page buttons
    int numPages = this->pages.size();

    float pageWidth = 480;
    if(numPages > 1) pageWidth /= numPages - 1;

    for (int i = 0; i < this->buttons.size(); i++){
        if(this->buttons.at(i).type == buttonType::page) this->buttons.at(i).Button = button(this->Screen, this->buttons.at(i).page_buttons_data.pageNum * pageWidth, 1, pageWidth, this->buttons.at(i).page_buttons_data.height, this->buttons.at(i).page_buttons_data.backgroundColor, this->buttons.at(i).page_buttons_data.outlineColor, this->buttons.at(i).page_buttons_data.outlineThickness, this->buttons.at(i).page_buttons_data.textColor, this->buttons.at(i).buttonName);
    }

    //draw main page
    const char* currentPage = "none";

    this->Screen->clearScreen(this->main_screen.background);

    for(int i = 0; i < this->buttons.size(); i++){
        if(this->buttons.at(i).page == currentPage) this->buttons.at(i).Button.draw();
    }

    bool breakPressed = false;

    // main loop
    while(!breakPressed){
        //wait until screen is pressed
        do { vex::task::sleep(2); } while (!this->Screen->pressing());
        
        //update buttons based on press
        for (int i = 0; i < this->buttons.size(); i++){
            if(this->buttons.at(i).Button.isPressing() && (this->buttons.at(i).page == currentPage || this->buttons.at(i).page == this->pages.at(0))){
                //button is pressed
                if(this->buttons.at(i).type == buttonType::page){
                    // update page selection
                    this->buttons.at(i).page_buttons_data.isSelected = !this->buttons.at(i).page_buttons_data.isSelected;
                    
                    //update if now selected
                    if(this->buttons.at(i).page_buttons_data.isSelected){
                        this->buttons.at(i).Button.changeColor(this->buttons.at(i).page_buttons_data.selectedBackgroundColor, this->buttons.at(i).page_buttons_data.selectedOutlineColor, this->buttons.at(i).page_buttons_data.selectedTextColor);
                        currentPage = this->buttons.at(i).buttonName;
                    }
                    else{
                        this->buttons.at(i).Button.changeColor(this->buttons.at(i).page_buttons_data.backgroundColor, this->buttons.at(i).page_buttons_data.outlineColor, this->buttons.at(i).page_buttons_data.textColor);
                        currentPage = this->pages.at(0);
                    }

                    //update other pages
                    for (int j = 0; j < this->buttons.size(); j++){
                        if(this->buttons.at(j).type == buttonType::page && i != j){
                            this->buttons.at(j).Button.changeColor(this->buttons.at(j).page_buttons_data.backgroundColor, this->buttons.at(j).page_buttons_data.outlineColor, this->buttons.at(j).page_buttons_data.textColor);
                            this->buttons.at(j).page_buttons_data.isSelected = false;
                        }
                    }
                }
                else if (this->buttons.at(i).type == buttonType::route){
                    //update auton selection
                    this->buttons.at(i).auton_buttons_data.isSelected = !this->buttons.at(i).auton_buttons_data.isSelected;

                    //update if now selected
                    if(this->buttons.at(i).auton_buttons_data.isSelected) this->buttons.at(i).Button.changeColor(this->buttons.at(i).auton_buttons_data.selectedBackgroundColor, this->buttons.at(i).auton_buttons_data.selectedOutlineColor, this->buttons.at(i).auton_buttons_data.selectedTextColor);
                    else this->buttons.at(i).Button.changeColor(this->buttons.at(i).auton_buttons_data.backgroundColor, this->buttons.at(i).auton_buttons_data.outlineColor, this->buttons.at(i).auton_buttons_data.textColor);

                    //update other auton buttons
                    for(int j = 0; j < this->buttons.size(); j++){
                        if(this->buttons.at(j).type == buttonType::route && i != j){
                            this->buttons.at(j).Button.changeColor(this->buttons.at(j).auton_buttons_data.backgroundColor, this->buttons.at(j).auton_buttons_data.outlineColor, this->buttons.at(j).auton_buttons_data.textColor);
                            this->buttons.at(j).auton_buttons_data.isSelected = false;
                        }
                    }
                }
                else if (this->buttons.at(i).type == buttonType::color){
                    //update color buttons
                    isRed = !isRed;

                    //update all color buttons
                    if(isRed == this->color_defaults.isRed){
                        for(int j = 0; j < this->buttons.size(); j++){
                            if(this->buttons.at(j).type == buttonType::color){
                                this->buttons.at(j).Button.changeColor(this->buttons.at(j).color_buttons_data.defaultColor, this->buttons.at(j).color_buttons_data.defaultOutlineColor, this->buttons.at(j).color_buttons_data.defaultTextColor);
                            }
                        }
                    }
                    else{
                        for(int j = 0; j < this->buttons.size(); j++){
                            if(this->buttons.at(j).type == buttonType::color){
                                this->buttons.at(j).Button.changeColor(this->buttons.at(j).color_buttons_data.secondaryColor, this->buttons.at(j).color_buttons_data.secondaryOutlineColor, this->buttons.at(j).color_buttons_data.secondaryTextColor);
                            }
                        }
                    }
                }
                else if (this->buttons.at(i).type == buttonType::__break){
                    //done
                    breakPressed = true;
                }
            }
        }

        //update display
        this->Screen->clearScreen(this->main_screen.background);

        for (int i = 0; i < this->buttons.size(); i++){
            if(this->buttons.at(i).page == currentPage || this->buttons.at(i).page == this->pages.at(0)) this->buttons.at(i).Button.draw();
        }

        //wait until screen is released
        do { vex::task::sleep(2); } while (this->Screen->pressing());
    }

    //get location of selected auton
    int location;
    for(location = 0; location < this->buttons.size(); location++){
        if(this->buttons.at(location).type == buttonType::route){
            if(this->buttons.at(location).auton_buttons_data.isSelected) break;
        }
    }

    if(location == this->buttons.size()) return {"none", "none"};
    else return {this->buttons.at(location).page, this->buttons.at(location).buttonName};
}

/**
 * Default constructor for button_data
 * dumb solution to a dumb problem
 */
selector::button_data::button_data(){}