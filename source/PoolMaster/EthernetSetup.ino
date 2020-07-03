#include <TextFinder.h>

// This is our buffer through which we will will "flow" our HTML code.
// It has to be as big as the longest character chain +1 including the "
char buffer[100];

// This is the HTML code all chopped up. The best way to do this is, is by typing
// your HTML code in an editor, counting your characters and divide them by 8.
// you can chop your HTML on every place, but not on the \" parts. So remember,
// you have to use \" instead of simple " within the HTML, or it will not work.

const char htmlx0[] PROGMEM = "<html><title>PoolMaster Ethernet Setup Page</title><body marginwidth=\"0\" marginheight=\"0\" ";
const char htmlx1[] PROGMEM = "leftmargin=\"0\" style=\"margin: 0; padding: 0;\"><table bgcolor=\"#999999\" border";
const char htmlx2[] PROGMEM = "=\"0\" width=\"100%\" cellpadding=\"1\" style=\"font-family:Verdana;color:#fff";
const char htmlx3[] PROGMEM = "fff;font-size:12px;\"><tr><td>&nbsp Poolmaster Ethernet Setup Page</td></tr></table><br>";
PGM_P const string_table0[] PROGMEM = {htmlx0, htmlx1, htmlx2, htmlx3};

// MAC is entered in HEX (T1 3 5 7 9 11) but submitted in (hidden) Decimal - Byte (0-254) (T2 4 6 8 10)

const char htmla0[] PROGMEM = "<script>function hex2num (s_hex) {eval(\"var n_num=0X\" + s_hex);return n_num;}";
const char htmla1[] PROGMEM = "</script><table><form><input type=\"hidden\" name=\"SBM\" value=\"1\"><tr><td>MAC:";
const char htmla2[] PROGMEM = "<input id=\"T1\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT1\" value=\"";
const char htmla3[] PROGMEM = "\">.<input id=\"T3\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT2\" value=\"";
const char htmla4[] PROGMEM = "\">.<input id=\"T5\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT3\" value=\"";
const char htmla5[] PROGMEM = "\">.<input id=\"T7\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT4\" value=\"";
const char htmla6[] PROGMEM = "\">.<input id=\"T9\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT5\" value=\"";
const char htmla7[] PROGMEM = "\">.<input id=\"T11\" type=\"text\" size=\"2\" maxlength=\"2\" name=\"DT6\" value=\"";
PGM_P const string_table1[] PROGMEM = {htmla0, htmla1, htmla2, htmla3, htmla4, htmla5, htmla6, htmla7};

const char htmlb0[] PROGMEM = "\"><input id=\"T2\" type=\"hidden\" name=\"DT1\"><input id=\"T4\" type=\"hidden\" name=\"DT2";
const char htmlb1[] PROGMEM = "\"><input id=\"T6\" type=\"hidden\" name=\"DT3\"><input id=\"T8\" type=\"hidden\" name=\"DT4";
const char htmlb2[] PROGMEM = "\"><input id=\"T10\" type=\"hidden\" name=\"DT5\"><input id=\"T12\" type=\"hidden\" name=\"D";
const char htmlb3[] PROGMEM = "T6\"></td></tr><tr><td>IP: <input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT7\" value=\"";
const char htmlb4[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT8\" value=\"";
const char htmlb5[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT9\" value=\"";
const char htmlb6[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT10\" value=\"";
PGM_P const string_table2[] PROGMEM = {htmlb0, htmlb1, htmlb2, htmlb3, htmlb4, htmlb5, htmlb6};

const char htmlc0[] PROGMEM = "\"></td></tr><tr><td>MASK: <input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT11\" value=\"";
const char htmlc1[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT12\" value=\"";
const char htmlc2[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT13\" value=\"";
const char htmlc3[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT14\" value=\"";
PGM_P const string_table3[]  PROGMEM = {htmlc0, htmlc1, htmlc2, htmlc3};

const char htmld0[] PROGMEM = "\"></td></tr><tr><td>GW: <input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT15\" value=\"";
const char htmld1[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT16\" value=\"";
const char htmld2[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT17\" value=\"";
const char htmld3[] PROGMEM = "\">.<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"DT18\" value=\"";
const char htmld4[] PROGMEM = "\"></td></tr><tr><td><br></td></tr><tr><td><input id=\"button1\"type=\"submit\" value=\"SUBMIT\" ";
const char htmld5[] PROGMEM = "></td></tr></form></table></body></html>";
PGM_P const string_table4[] PROGMEM = {htmld0, htmld1, htmld2, htmld3, htmld4, htmld5};

// MAC is entered in HEX (T1 3 5 7 9 11) but submitted in (hidden) Decimal - Byte (0-254) (T2 4 6 8 10)

const char htmle0[] PROGMEM = "Onclick=\"document.getElementById('T2').value ";
const char htmle1[] PROGMEM = "= hex2num(document.getElementById('T1').value);";
const char htmle2[] PROGMEM = "document.getElementById('T4').value = hex2num(document.getElementById('T3').value);";
const char htmle3[] PROGMEM = "document.getElementById('T6').value = hex2num(document.getElementById('T5').value);";
const char htmle4[] PROGMEM = "document.getElementById('T8').value = hex2num(document.getElementById('T7').value);";
const char htmle5[] PROGMEM = "document.getElementById('T10').value = hex2num(document.getElementById('T9').value);";
const char htmle6[] PROGMEM = "document.getElementById('T12').value = hex2num(document.getElementById('T11').value);\"";
PGM_P const string_table5[] PROGMEM = {htmle0, htmle1, htmle2, htmle3, htmle4, htmle5, htmle6};

//Ethernet client checking loop (the Web server sending the ethernet setup webpage to the browser)
//call http://PoolMaster.local to load the page on your LAN
void EthernetClientCallback(Task * me)
{
  EthernetClient client = server.available();
  if (client) {
    TextFinder  finder(client );
    while (client.connected()) {
      if (client.available()) {
        //This part does all the text searching
        if ( finder.find("GET /") ) {

          // if you find the word "setup" continue looking for more
          // if you don't find that word, stop looking and go further
          // This way you can put your own webpage later on in the sketch
          if (finder.findUntil("setup", "\n\r")) {

            // if you find the word "SBM" continue looking for more
            // if you don't find that word, stop looking and go further
            // it means the SUBMIT button hasn't been pressed an nothing has
            // been submitted. Just go to the place where the setup page is
            // been build and show it in the client's browser.
            if (finder.findUntil("SBM", "\n\r")) {

              byte SET = finder.getValue();
              // Now while you are looking for the letters "DT", you'll have to remember
              // every number behind "DT" and put them in "val" and while doing so, check
              // for the according values and put those in mac, ip, subnet and gateway.
              while (finder.findUntil("DT", "\n\r")) {
                int val = finder.getValue();
                // if val from "DT" is between 1 and 6 the according value must be a MAC value.
                if (val >= 1 && val <= 6) {
                  storage.mac[val - 1] = finder.getValue();
                }
                // if val from "DT" is between 7 and 10 the according value must be a IP value.
                if (val >= 7 && val <= 10) {
                  storage.ip[val - 7] = finder.getValue();
                }
                // if val from "DT" is between 11 and 14 the according value must be a MASK value.
                if (val >= 11 && val <= 14) {
                  storage.subnet[val - 11] = finder.getValue();
                }
                // if val from "DT" is between 15 and 18 the according value must be a GW value.
                if (val >= 15 && val <= 18) {
                  storage.gateway[val - 15] = finder.getValue();
                }
              }
              // Now that we got all the data, we can save it to EEPROM
              Serial << F("Pressed SUBMIT button") << endl;
              storage.ipConfiged = 1;
              saveConfig();
              while(true){};//restart
            }
            // and from this point on, we can start building our setup page
            // and show it in the client's browser.
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println();
            for (int i = 0; i < 4; i++)
            {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
              client.print( buffer );
            }
            for (int i = 0; i < 3; i++)
            {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[i])));
              client.print( buffer );
            }
            client.print(storage.mac[0], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[3])));
            client.print( buffer );
            client.print(storage.mac[1], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[4])));
            client.print( buffer );
            client.print(storage.mac[2], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[5])));
            client.print( buffer );
            client.print(storage.mac[3], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[6])));
            client.print( buffer );
            client.print(storage.mac[4], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[7])));
            client.print( buffer );
            client.print(storage.mac[5], HEX);
            for (int i = 0; i < 4; i++)
            {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[i])));
              client.print( buffer );
            }
            client.print(storage.ip[0], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[4])));
            client.print( buffer );
            client.print(storage.ip[1], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[5])));
            client.print( buffer );
            client.print(storage.ip[2], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[6])));
            client.print( buffer );
            client.print(storage.ip[3], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[0])));
            client.print( buffer );
            client.print(storage.subnet[0], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[1])));
            client.print( buffer );
            client.print(storage.subnet[1], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[2])));
            client.print( buffer );
            client.print(storage.subnet[2], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[3])));
            client.print( buffer );
            client.print(storage.subnet[3], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[0])));
            client.print( buffer );
            client.print(storage.gateway[0], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[1])));
            client.print( buffer );
            client.print(storage.gateway[1], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[2])));
            client.print( buffer );
            client.print(storage.gateway[2], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[3])));
            client.print( buffer );
            client.print(storage.gateway[3], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[4])));
            client.print( buffer );
            for (int i = 0; i < 7; i++)
            {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table5[i])));
              client.print( buffer );
            }
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[5])));
            client.print( buffer );

            // Show if default configuration has been changed

            if (!storage.ipConfiged) {
              client.print("<br /> <br />");
              client.print("Press submit to change deault IP configuration" );
              client.print("<br /> <br />");
            }
            else
            {
              client.print("<br /> <br />");
              client.print("Default IP configuration has been updated!" );
              client.print("<br />");
              client.print("Restarting Poolmaster... Wait for 20 secs and refresh page." );
              client.print("<br /> <br />");
            }

            break;
          }
        }
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        for (int i = 0; i < 4; i++)
        {
          strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
          client.print( buffer );
        }
        // put your own html from here on
        client.print(" !! POOLMASTER TEMPORARY IP ADDRRESS !!");
        client.print("<br /> <br />");
        client.print("go to:");
        client.print("<br /> <br />");
        client.print("http://");
        client.print(storage.ip[0], DEC);
        for (int i = 1; i < 4; i++) {
          client.print(".");
          client.print(storage.ip[i], DEC);
        }
        client.print("/setup");
        client.print("<br /> <br />");
        client.print("to configure an IP ADDRESS for your Poolmaster on your network");
        client.print("<br /> <br />");
        client.print("<a href=");
        client.print("https://github.com/Loic74650/PoolMaster");
        client.println(">Poolmaster github repository</a>");
        // put your own html until here
        break;
      }
    }
    delay(1);
    client.stop();
  }
}
