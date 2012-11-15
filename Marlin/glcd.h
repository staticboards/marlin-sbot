# ifndef STATICBOT_INCLUDE
# define STATICBOT_INCLUDE

# include "Configuration.h"

extern void glcd_setpixel ( const uint8_t x, const uint8_t y, const uint8_t color ) ;

extern void glcd_drawrect ( const uint8_t x , const uint8_t y , const uint8_t w , const uint8_t h , const uint8_t color ) ;
extern void glcd_drawline ( uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t color ) ;

extern void glcd_drawbitmap ( const uint8_t x , const uint8_t y, const uint8_t bitmap, const uint8_t w, const uint8_t h , const uint8_t color ) ;
extern void glcd_drawchar   ( const uint8_t x , const uint8_t line, const char c ) ;
extern void glcd_drawstring (uint8_t x, uint8_t line, const char *c) ;

extern void glcd_drawcircle ( const uint8_t x0, const uint8_t y0, const uint8_t r, const uint8_t color ) ;

extern void glcd_fillrect ( const uint8_t x, const uint8_t y, const uint8_t w, const uint8_t h, const uint8_t color ) ;

extern void glcd_clearbuffer(void) ;

extern void glcd_init (void ) ;
extern void glcd_noop (void ) ;
extern void glcd_sendbuffer (void ) ;

extern void glcd_set_brightness(const uint8_t val) ;
extern  uint8_t glcd_get_brightness (void ) ;

class glcd
{

private :
  uint8_t xpos , ypos ;
  char text [ LCD_WIDTH*LCD_HEIGHT ] ;  
  uint8_t dirty ;

public :

	glcd ()
	{
		this->xpos = this->ypos = this->dirty = 0 ;
	}

	void begin (int , int )
	{
		memset ( this->text , 0x0 , LCD_WIDTH*LCD_HEIGHT ) ;
		glcd_init ();
	}

	void createChar( const uint8_t , const byte * )
	{

	}

	void upload (void )
	{
	  if ( ! this->dirty ) return ;
	  
	  uint8_t x,line ;
	  uint8_t c ;

	  char * p = this->text ;
	  
	  for (line = 0 ; line<LCD_HEIGHT ; ++ line )
	    {
	      c = 0 ;
	      for ( x = 0 ; x < LCD_WIDTH ; ++ x , c+=5 )
		{
		  glcd_drawchar(c, line, *p++);
		}
	    }
	  glcd_sendbuffer () ;
	  this->dirty = 0 ;
	}


	void setCursor ( const uint8_t x , const uint8_t y )
	{
	  this->xpos = (x>=LCD_WIDTH?LCD_WIDTH-1:x) ;
	  this->ypos = (y>=LCD_HEIGHT?LCD_HEIGHT-1:y) ;
	}  

	void clear (void )
	{
	  memset ( this->text , ' ' , LCD_WIDTH * LCD_HEIGHT ) ;
	  this->xpos = 0 ;
	  this->ypos = 0 ;
	  this->dirty = 1 ;
	}

	void print ( const char ch )
	{  
	  char *p = this->text + (LCD_WIDTH*this->ypos) + this->xpos ;
	  *p = ch ;
	  ++ this->xpos ;
	  this->dirty = 1 ;
	}

	void print ( const char * msg )
	{
      if (!msg) return;
	  char *p = this->text + (LCD_WIDTH*this->ypos) + this->xpos ;
	  
	  while ( (*msg) && (this->xpos < LCD_WIDTH) )
	    {
	      *p++ = *msg++ ;
	      ++ this->xpos ;
	      this->dirty = 1 ;
	    }  
	}

} ;

# endif // STATICBOT_INCLUDE
