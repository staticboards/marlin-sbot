# include <math.h>
# include <stdint.h>
# include <avr/io.h>
# include <util/delay.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// I2C SUPPORT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//TODO: COMPROBAR SI LOS VALORES DE PULL-UP DEL AVR (20-50k?) FUNCIONAN BIEN EN LA PLACA (SE PRUEDE PROBAR CON ESTOS VALORES EXTREMOS) Y VER EL dV

/*
TODO:
The Power Reduction TWI bit, PRTWI bit in .Power Reduction Register 0 - PRR0. on page 54
must be written to zero to enable the 2-wire Serial Interface.
*/

#define I2CMAXRETRIES 3

#define SCLFREQ2TWBR(f) ((F_CPU / (f) - 16) / 2)
#define SCLFREQ         200000UL // Max 222.222kHz with 8 MHz oscillator

enum write_status_e {
	WS_SEND_START,
	WS_SEND_I2CADDRESS,
	WS_SEND_ADDRESS,
	WS_SEND_DATA,
	WS_SEND_STOP,
	WS_UNEXPECTED,
	WS_NACK_RECEIVED,
	WS_OK,
	WS_FAILED
};

enum read_status_e {
	RS_SEND_START,
	RS_SEND_I2CADDRESS_W,
	RS_SEND_ADDRESS,
	RS_SEND_RSTART,
	RS_SEND_I2CADDRESS_R,
	RS_RECV_DATA,
	RS_SEND_STOP,
	RS_UNEXPECTED,
	RS_NACK_RECEIVED,
	RS_OK,
	RS_FAILED
};

enum hwstatus_e {
	START_TRANSMITTED = 0x08,
	REPEATED_START_TRANSMITTED = 0x10,
	ARBITRATION_LOST = 0x38,
 
	SLA_W_TRANSMITTED_ACK = 0x18,
	SLA_W_TRANSMITTED_NACK = 0x20,
	DATA_TRANSMITTED_ACK = 0x28,
	DATA_TRANSMITTED_NACK = 0x30,

	SLA_R_TRANSMITTED_ACK = 0x40,
	SLA_R_TRANSMITTED_NACK = 0x48,
	DATA_RECEIVED_ACK = 0x50,
	DATA_RECEIVED_NACK = 0x58
};



void i2c_recover ( void )
{
	/* Configure SDA as INPUT, SCL as output */

	/* Disable I2C */
	TWCR = 0 ;

	/* Clock SCL and wait for SDA = 1 */

	/* Send I2C STOP manually */

	/* *** BUS IS FREE NOW *** */
}


void i2c_init ( void )
{
	/* Configure I/O and enable pull-ups *///F_CPU!

	/* Reset pending transmissions */ 

	/* Set clock speed */
	TWSR = 0 ; /* Prescaler 1:1 */
	TWBR = SCLFREQ2TWBR ( SCLFREQ ) ; /* TWBR should *always* be >= 10 */

	/* Enable TWI */
	TWCR = _BV ( TWEN ) ;
}


inline void i2c_cmd_start ( void )
{
	TWCR = _BV ( TWINT ) | _BV ( TWSTA ) | _BV ( TWEN ) ;
}


inline void i2c_cmd_data ( void )
{
	TWCR = _BV ( TWINT ) | _BV ( TWEN ) ;
}


inline void i2c_cmd_recv ( uint8_t ack )
{
	TWCR = _BV ( TWINT ) | _BV ( TWEN ) | ( ack ? _BV ( TWEA ) : 0 ) ;
}


inline uint8_t i2c_completed ( void )
{
	return bit_is_set ( TWCR , TWINT ) ;
}


inline uint8_t i2c_status ( void )
{
	return TWSR & 0xf8 ;
}


inline void i2c_cmd_stop ( void )
{
	TWCR = _BV ( TWINT ) | _BV ( TWSTO ) | _BV ( TWEN ) ;
}


uint8_t i2c_write_a1 ( uint8_t i2caddress , uint8_t address , const uint8_t * data , uint8_t size )
{
  uint8_t retries = 0 ;
  enum write_status_e status = (write_status_e) WS_SEND_START ;
  uint8_t i = 0 ;

  do 
    {
      switch ( status ) 
	{

	case WS_SEND_START :
	  i = 0 ;
	  i2c_cmd_start () ;
	  while ( ! i2c_completed () ) ;
	  if ( i2c_status () == START_TRANSMITTED )
	    status = WS_SEND_I2CADDRESS ;
	  else
	    status = WS_UNEXPECTED ;
	  break ;
		
	case WS_SEND_I2CADDRESS :
	  TWDR = i2caddress & 0xfe ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed() ) ;
	  switch ( i2c_status () ) 
	    {
	    case SLA_W_TRANSMITTED_ACK :
	      status = WS_SEND_ADDRESS ;
	      break ;
	    case SLA_W_TRANSMITTED_NACK :
	      status = WS_NACK_RECEIVED ;
	      break ;
	    case ARBITRATION_LOST :
	    default :
	      status = WS_UNEXPECTED ;
	      break ;
	    }
	  break ;
		
	case WS_SEND_ADDRESS :
	  TWDR = address ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed () ) ;
	  switch ( i2c_status () ) 
	    {
	    case DATA_TRANSMITTED_ACK :
	      if ( i == size )
		status = WS_SEND_STOP ;
	      else
		status = WS_SEND_DATA ;
	      break ;
	    case DATA_TRANSMITTED_NACK :
	      status = WS_NACK_RECEIVED ;
	      break ;
	    case ARBITRATION_LOST :
	    default :
	      status = WS_UNEXPECTED ;
	      break ;
	    }
	  break ;
		
	case WS_SEND_DATA :
	  TWDR = data [ i ] ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed () ) ;
	  switch ( i2c_status () ) 
	    {
	    case DATA_TRANSMITTED_ACK :
	      i ++ ;
	      if ( i == size )
		status = WS_SEND_STOP ;
	      /* status left unchanged if there is more data to transmit */
	      break ;
	    case DATA_TRANSMITTED_NACK :
	      status = WS_NACK_RECEIVED ;
	      break ;
	    case ARBITRATION_LOST :
	    default :
	      status = WS_UNEXPECTED ;
	      break ;
	    }
	  break ;
		
	case WS_SEND_STOP :
	  i2c_cmd_stop () ;
	  /* Habra que esperar ????? */
	  status = WS_OK ;
	  break ;
		
	case WS_NACK_RECEIVED :
	  i2c_cmd_stop () ;
	  /* Esperar??? */
	  retries ++ ;
	  if ( retries <= I2CMAXRETRIES) 
	    {
	      status = WS_SEND_START;
	    } 
	  else 
	    {
	      status = WS_FAILED;
	    }
	  break ;
		
	case WS_FAILED :
	  //TODO: ?
	  break ;

	case WS_UNEXPECTED :
	  //TODO: Esto no deberia pasar nunca
	  break ;

	default :
	  break ;
	}
	    
    } 
  while ( ( status != WS_OK ) && ( status != WS_FAILED ) ) ;
	
  if ( status == WS_OK )
    return 1 ;
  else
    return 0 ;
}


uint8_t i2c_read_a1 ( uint8_t i2caddress , uint8_t address , uint8_t * data , uint8_t size )
{
  uint8_t retries = 0 ;
  enum read_status_e status = (read_status_e ) RS_SEND_START ;
  uint8_t i = 0;
  
  do 
    {
      switch ( status )
	{
	  
	case RS_SEND_START :
	  i = 0 ;
	  i2c_cmd_start () ;
	  while ( ! i2c_completed () ) ;
	  if ( i2c_status () == START_TRANSMITTED )
	    status = RS_SEND_I2CADDRESS_W ;
	  else
	    status = RS_UNEXPECTED ;
	  break ;
		
	case RS_SEND_I2CADDRESS_W :
		
	  TWDR = i2caddress & 0xfe ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed () ) ;
	  switch ( i2c_status () ) {
	  case SLA_W_TRANSMITTED_ACK :
	    status = RS_SEND_ADDRESS ;
	    break ;
	  case SLA_W_TRANSMITTED_NACK :
	    status = RS_NACK_RECEIVED ;
	    break ;
	  case ARBITRATION_LOST :
	  default :
	    status = RS_UNEXPECTED ;
	    break ;
	  }
	  break ;
			
	case RS_SEND_ADDRESS :
	  TWDR = address ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed () ) ;
	  switch (i2c_status () ) 
	    {
	    case DATA_TRANSMITTED_ACK :
	      if ( i == size )
		status = RS_SEND_STOP ;
	      else
		status = RS_SEND_RSTART ;
	      break ;
	    case DATA_TRANSMITTED_NACK :
	      status = RS_NACK_RECEIVED ;
	      break ;
	    case ARBITRATION_LOST :
	    default :
	      status = RS_UNEXPECTED ;
	      break ;
	    }
	  break ;
			
	case RS_SEND_RSTART :
	  i2c_cmd_start () ;
	  while ( ! i2c_completed () ) ;
	  if ( i2c_status () == REPEATED_START_TRANSMITTED )
	    status = RS_SEND_I2CADDRESS_R ;
	  else
	    status = RS_UNEXPECTED ;
	  break;
		
	case RS_SEND_I2CADDRESS_R :
		
	  TWDR = i2caddress | 0x01 ;
	  i2c_cmd_data () ;
	  while ( ! i2c_completed () ) ;
	  switch ( i2c_status () ) 
	    {
	    case SLA_R_TRANSMITTED_ACK :
	      status = RS_RECV_DATA ;
	      break ;
	    case SLA_R_TRANSMITTED_NACK :
	      status = RS_NACK_RECEIVED ;
	      break ;
	    case ARBITRATION_LOST :
	    default :
	      status = RS_UNEXPECTED ;
	      break ;
	    }
	  break ;
		
	case RS_RECV_DATA :
	  /* Last byte read is nacked */
	  i2c_cmd_recv ( ( i + 1 ) != size ) ;
	  while ( ! i2c_completed () ) ;
	  switch ( i2c_status () ) {
	  case DATA_RECEIVED_ACK :
	    data [ i ++ ] = TWDR ;
	    /* status left unchanged as there is more data to receive */
	    break ;
	  case DATA_RECEIVED_NACK :
	    /* Last byte received */
	    data [ i ++ ] = TWDR ;
	    status = RS_SEND_STOP ;
	    break ;
	  case ARBITRATION_LOST :
	  default :
	    status = RS_UNEXPECTED ;
	    break ;
	  }
	  break ;
		
	case RS_SEND_STOP :
	  i2c_cmd_stop () ;
	  /* Habra que esperar ????? */
	  status = RS_OK ;
	  break ;

	case RS_NACK_RECEIVED:
	  i2c_cmd_stop () ;
	  /* Esperar??? */
	  retries ++ ;
	  if ( retries <= I2CMAXRETRIES ) 
	    {
	      status = RS_SEND_START ;
	    } 
	  else 
	    {
	      status = RS_FAILED ;
	    }
	  break ;
		
	case RS_UNEXPECTED :
	  break ;
	}
	    
    } while ( ( status != RS_OK ) && ( status != RS_FAILED ) ) ;

  if ( status == RS_OK )
    return 1 ;
  else
    return 0 ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

# define TCA62724FMG_SLAVE_ADDRESS_READ   0b10101011
# define TCA62724FMG_SLAVE_ADDRESS_WRITE  0b10101010


# define TCA62724FMG_CMD_PWM_DUTYDATASETUP_OUT0     1
# define TCA62724FMG_CMD_PWM_DUTYDATASETUP_OUT1     2
# define TCA62724FMG_CMD_PWM_DUTYDATASETUP_OUT2     3
# define TCA62724FMG_CMD_ENABLE                     4

# define TCA62724FMG_CMD_AUTOINCREMENT_OFF          1<<7
# define TCA62724FMG_CMD_AUTOINCREMENT_ON           0<<7

# define TCA62724FMG_DATA_SHDN                      1<<0
# define TCA62724FMG_DATA_ENABLE                    1<<1

uint8_t tca62724fmg_set_out ( const uint8_t idx , const uint8_t value )
{
  return i2c_write_a1 ( TCA62724FMG_SLAVE_ADDRESS_WRITE , TCA62724FMG_CMD_AUTOINCREMENT_OFF | (TCA62724FMG_CMD_PWM_DUTYDATASETUP_OUT0+idx) , (const uint8_t *) &value , sizeof(value) ) ;
}

uint8_t tca62724fmg_enable (void )
{
  const uint8_t value = TCA62724FMG_DATA_ENABLE | TCA62724FMG_DATA_SHDN ;

  return i2c_write_a1 ( TCA62724FMG_SLAVE_ADDRESS_WRITE , TCA62724FMG_CMD_AUTOINCREMENT_OFF | TCA62724FMG_CMD_ENABLE , (const uint8_t *) &value , sizeof(value) ) ;
}

static uint8_t tca62724fmg_data [4];

uint8_t tca62724fmg_setcolor ( const uint8_t r , const uint8_t g , const uint8_t b )
{

  tca62724fmg_data [0] = b & 0xf ;
  tca62724fmg_data [1] = g & 0xf ;
  tca62724fmg_data [2] = r & 0xf ;
  tca62724fmg_data [3] = TCA62724FMG_DATA_ENABLE | TCA62724FMG_DATA_SHDN ;

  return i2c_write_a1 ( TCA62724FMG_SLAVE_ADDRESS_WRITE , TCA62724FMG_CMD_AUTOINCREMENT_ON | TCA62724FMG_CMD_PWM_DUTYDATASETUP_OUT0 , tca62724fmg_data , sizeof(tca62724fmg_data) ) ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;

static rgb      hsv2rgb(hsv in);

rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        if(isnan(in.h)) {   // in.h == NAN
            out.r = in.v;
            out.g = in.v;
            out.b = in.v;
            return out;
        }
        // error - should never happen
        out.r = 0.0;
        out.g = 0.0;
        out.b = 0.0;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

uint8_t tca62724fmg_setrgb ( const double &red , const double &green, const double &blue )
{
      const uint8_t r = (const uint8_t) ( 15.99 * red ) ;
      const uint8_t g = (const uint8_t) ( 15.99 * green ) ;
      const uint8_t b = (const uint8_t) ( 15.99 * blue ) ;

      return tca62724fmg_setcolor (r,g,b) ;
}

uint8_t tca62724fmg_sethsv ( const double &hue , const double &saturation, const double &value )
{
	const hsv color = { hue , saturation , value } ;

      const rgb tmp = hsv2rgb(color) ;

      const uint8_t r = (const uint8_t) ( 15.99 * tmp.r ) ;
      const uint8_t g = (const uint8_t) ( 15.99 * tmp.g ) ;
      const uint8_t b = (const uint8_t) ( 15.99 * tmp.b ) ;

      return tca62724fmg_setcolor (r,g,b) ;
}

# define TCA_HUE_P0		0
# define TCA_HUE_P1		0
# define TCA_HUE_P2		120
# define TCA_HUE_P3		120

# define TCA_SAT_P0		1
# define TCA_SAT_P1		1
# define TCA_SAT_P2		1
# define TCA_SAT_P3		0

# define TCA_VAL_P0		0
# define TCA_VAL_P1		0.5
# define TCA_VAL_P2		0.75
# define TCA_VAL_P3		1

# define TCA_P1 		0.60
# define TCA_P2 		0.95
# define TCA_P3 		1

#define TCAREMAP(v0,v1,t)	(((t ) - (v0)) / ((v1) - (v0)))
#define TCALERP(v0,v1,t)	(((v1) - (v0)) * (t) + (v0))

uint8_t tca62724fmg_showpercent ( const double & percent )
{
	if ( percent < TCA_P1 )
	{
		const double t = TCAREMAP(0,TCA_P1,percent) ;
		const double hue = TCALERP ( TCA_HUE_P0 , TCA_HUE_P1 , t ) ;
		const double sat = TCALERP ( TCA_SAT_P0 , TCA_SAT_P1 , t ) ;
		const double val = TCALERP ( TCA_VAL_P0 , TCA_VAL_P1 , t ) ;

		return tca62724fmg_sethsv ( hue , sat , val ) ;
	}
	else if ( percent < TCA_P2 )
	{
		const double t = TCAREMAP(TCA_P1,TCA_P2,percent) ;
		const double hue = TCALERP ( TCA_HUE_P1 , TCA_HUE_P2 , t ) ;
		const double sat = TCALERP ( TCA_SAT_P1 , TCA_SAT_P2 , t ) ;
		const double val = TCALERP ( TCA_VAL_P1 , TCA_VAL_P2 , t ) ;

		return tca62724fmg_sethsv ( hue , sat , val ) ;
	}
	else if ( percent < TCA_P3 )
	{
		const double t = TCAREMAP(TCA_P2,TCA_P3,percent) ;
		const double hue = TCALERP ( TCA_HUE_P2 , TCA_HUE_P3 , t ) ;
		const double sat = TCALERP ( TCA_SAT_P2 , TCA_SAT_P3 , t ) ;
		const double val = TCALERP ( TCA_VAL_P2 , TCA_VAL_P3 , t ) ;

		return tca62724fmg_sethsv ( hue , sat , val ) ;		
	}
	else
	{
		return tca62724fmg_sethsv ( TCA_HUE_P3 , TCA_SAT_P3 , TCA_VAL_P3 ) ;				
	}
}

void tca62724fmg_init (void )
{	
	i2c_init () ;

	_delay_ms (50);	
	tca62724fmg_setcolor ( 0xf,0,0) ;
	_delay_ms (1000);
	tca62724fmg_setcolor ( 0x0,0xf,0) ;
	_delay_ms (1000);
	tca62724fmg_setcolor ( 0x0,0x0,0xf) ;
	_delay_ms (1000);
	
	tca62724fmg_setcolor (0,0,0) ;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
