#ifndef TCA62724FMG_H
#define TCA62724FMG_H

// init
void tca62724fmg_init (void ) ;

// set the RGB led using RGB values
uint8_t tca62724fmg_setrgb ( const double &red , const double &green, const double &blue ) ;

// set the RGB led using HSV values
uint8_t tca62724fmg_sethsv ( const double &hue , const double &saturation, const double &value ) ;

// link a int value to a color
uint8_t tca62724fmg_showpercent ( const double & percent ) ;

#endif // TCA62724FMG_H