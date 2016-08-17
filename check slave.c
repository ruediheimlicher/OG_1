if (currentstatus & (1<<IMPULSBIT)) // neuer Impuls angekommen, Zaehlung lauft
{
   /*
    inbuffer[0]=0;
    inbuffer[1]=0;
    inbuffer[2]=0;
    in_startdaten=0;
    out_startdaten = 0xB1;
    */
   //OSZILO;
   //   PORTD |=(1<<ECHOPIN);
   currentstatus++; // ein Wert mehr gemessen
   messungcounter ++;
   impulszeitsumme += (impulszeit/ANZAHLWERTE);      // float, Wert aufsummieren
   
   
   // interger addieren
   integerimpulszeit += impulszeit; // float, float
   
   if (filtercount == 0) // neues Paket
   {
      filtermittelwert = impulszeit;
   }
   else
   {
      if (filtercount < filterfaktor)
      {
         filtermittelwert = ((filtercount-1)* filtermittelwert + impulszeit)/filtercount;
         
         //lcd_gotoxy(19,1);
         //lcd_putc('a');
      }
      else
      {
         filtermittelwert = ((filterfaktor-1)* filtermittelwert + impulszeit)/filterfaktor;
         //lcd_gotoxy(19,1);
         //lcd_putc('f');
      }
      
   }
   
   
   //            char filterstromstring[8];
   //            filtercount++;
   /*
    lcd_gotoxy(0,0);
    lcd_putint16(impulszeit/100);
    lcd_gotoxy(10,0);
    lcd_putint16(filtermittelwert/100);
    lcd_gotoxy(10,1);
    lcd_putint(filtercount);
    lcd_putc('*');
    
    dtostrf(filtermittelwert,5,1,filterstromstring);
    //lcd_puts(filterstromstring);
    */
   
   //         if (filtercount & (filterfaktor == 0)) // Wert anzeigen
   {
      //lcd_gotoxy(10,0);
      //lcd_putint16(filtermittelwert);
      //lcd_putc('*');
      //dtostrf(filtermittelwert,5,1,filterstromstring);
      //lcd_puts(filterstromstring);
   }
   
   
   if ((currentstatus & 0x0F) == ANZAHLWERTE)      // genuegend Werte
   {
      OSZILO;
      inbuffer[0]=0;
      inbuffer[1]=0;
      inbuffer[2]=0;
      in_startdaten=0;
      out_startdaten = 0xB1;
      outbuffer[0] = testwert;
      outbuffer[1] = testwert;
      outbuffer[2] = testwert;
      
      testwert++;
      
      //     currentstatus &= ~(1<<IMPULSBIT);
      //     currentstatus &= ~(1<<NEWBIT);
      //     currentstatus &= 0xF0; // Bit 0-3 reset
      
      //    continue;
      //               lcd_gotoxy(19,0);
      //               lcd_putc(' ');
      
      
      //lcd_putc(' ');
      //lcd_gotoxy(6,1);
      //lcd_putc(' ');
      
      //lcd_gotoxy(16,1);
      //lcd_puts("  \0");
      //lcd_gotoxy(0,1);
      //lcd_puts("    \0");
      
      //lcd_gotoxy(0,1);
      //lcd_putint(messungcounter);
      
      
      
      //lcd_gotoxy(0,0);
      //lcd_puts("  \0");
      /*
       if ((paketcounter & 1)==0)
       {
       lcd_gotoxy(8,1);
       lcd_putc(':');
       
       }
       else
       {
       lcd_gotoxy(8,1);
       lcd_putc(' ');
       }
       */
      paketcounter++;
      
      lcd_gotoxy(0,0);
      lcd_putint(paketcounter);
      cli();
      
      currentstatus &= 0xF0; // Bit 0-3 reset
      
      // Wert fuer SPI-Uebertragung
      
      impulsmittelwert = impulszeitsumme; // float = float
      
      // timer1 setzen
      OCR1A = (uint16_t)(impulsmittelwert+0.5); // float to uint16
      
      // summe resetten
      impulszeitsumme = 0;
      
      //               impulsmittelwertl = ((uint32_t)impulsmittelwert & 0xFF);
      //              impulsmittelwerth = ((uint32_t)impulsmittelwert>>8) & 0xFF;
      //             impulsmittelwerthh = ((uint32_t)impulsmittelwert>>16) & 0xFF;
      
      //               outbuffer[0] = ((uint32_t)impulsmittelwert & 0xFF);
      //               outbuffer[1] = ((uint32_t)impulsmittelwert>>8) & 0xFF;
      //              outbuffer[2] = ((uint32_t)impulsmittelwert>>16) & 0xFF;
      //out_startdaten = 0x13;
      
      sei();
      //                lcd_gotoxy(0,1);
      //               lcd_putc('I');
      //lcd_puts("INT0 \0");
      
      //lcd_puthex(hb);
      //lcd_puthex(lb);
      //lcd_putc(':');
      
      //char impstring[12];
      //dtostrf(impulsmittelwert,8,2,impstring);
      
      //            lcd_gotoxy(0,3);
      //           lcd_putint16(((uint16_t)impulsmittelwert) );
      //lcd_putc('*');
      
      
      // lcd_gotoxy(5,0);
      // lcd_putint(sendintervallzeit);
      // lcd_putc('$');
      
      /*
       Impulsdauer: impulsmittelwert * TIMERIMPULSDAUER (10us)
       Umrechnung auf ms: /1000
       Energie pro Zählerimpuls: 360 Ws
       Leistung: (Energie pro Zählerimpuls)/Impulsabstand
       Umrechnung auf Sekunden: *1000
       Faktor: *100000
       */
      
      //     leistung = 0xFFFF/impulsmittelwert;
      //              cli();
      
      // Leistung berechnen
      /*
       if (impulsmittelwert)
       {
       leistung = 360.0/impulsmittelwert*100000.0;// 480us
       
       // webleistung = (uint32_t)360.0/impulsmittelwert*1000000.0;
       webleistung = (uint32_t)360.0/impulsmittelwert*100000.0;
       
       
       //                 lcd_gotoxy(0,1);
       //                 lcd_putint16(webleistung);
       //                 lcd_putc('*');
       }
       wattstunden = impulscount/10; // 310us
       */
      
      //               sei();
      
      //     Stromzaehler
      //OSZILO;
      /*
       // ganze Anzeige 55 ms
       lcd_gotoxy(9,1);
       lcd_putint(wattstunden/1000);
       lcd_putc('.');
       lcd_putint3(wattstunden);
       lcd_putc('W');
       lcd_putc('h');
       */
      //OSZIHI;
      
      
      // dtostrf(leistung,5,0,stromstring); // fuehrt zu 'strom=++123' in URL fuer strom.pl. Funktionierte trotzdem
      
      //         dtostrf(leistung,5,1,stromstring); // 800us
      
      
      //lcd_gotoxy(0,0);
      //lcd_putc('L');
      //lcd_putc(':');
      
      
      //            if (!(paketcounter == 1))
      {
         
         //lcd_puts("     \0");
         
         //lcd_gotoxy(2,0);
         //lcd_puts(stromstring);
         //lcd_putc(' ');
         //lcd_putc('W');
      }
      //lcd_putc('*');
      //lcd_putc(' ');
      //lcd_putint16(leistung);
      //lcd_putc(' ');
      
      /*
       if (abs(leistung-lastleistung) > 10)
       {
       lastcounter++;
       
       if (lastcounter>3)
       {
       char diff[10];
       dtostrf(leistung-lastleistung,7,2,diff);
       lcd_gotoxy(10,1);
       lcd_putc('D');
       lcd_putc(':');
       lcd_puts(diff);
       lastleistung = leistung;
       }
       }
       else
       {
       lastcounter=0;
       }
       */
      
      // if (paketcounter  >= ANZAHLPAKETE)
      if (webstatus & (1<<DATALOOP))
      {
         
         webstatus &= ~(1<<DATALOOP);
         
         //uint16_t zufall = rand() % 0x0F + 1;;
         
         //lcd_putc(' ');
         //lcd_putint12(zufall);
         //leistung += zufall;
         
         
         
         //dtostrf(leistung,5,1,stromstring); // 800us
         
         dtostrf(webleistung,10,0,stromstring); // 800us
         
         
         paketcounter=0;
         
         /*
          uint16_t tempmitte = 0;
          for (i=0;i<4;i++)
          {
          tempmitte+= stromimpulsmittelwertarray[i];
          }
          tempmitte/= 4;
          */
         /*
          lcd_gotoxy(14,0);
          lcd_putc('m');
          lcd_putint12(tempmitte);
          */
         //         filtercount =0;
         
         //if (TEST)
         {
            //lcd_gotoxy(0,0);
            //lcd_putint(messungcounter);
            //lcd_putc(' ');
            //OSZILO;
            
            /*
             lcd_gotoxy(9,2);
             lcd_putint(wattstunden/1000);
             lcd_putc('.');
             lcd_putint3(wattstunden);
             //lcd_putc('W');
             //lcd_putc('h');
             */
            //OSZIHI;
         }
         
         
         // senden aktivieren
         webstatus |= (1<<DATASEND);
         webstatus |= (1<<DATAOK);
         // Messung anhalten
         webstatus |= (1<<CURRENTSTOP);
         // Warten aktivieren
         webstatus |= (1<<CURRENTWAIT);
         
         paketcounter=0;
         //sendWebCount++;
         //           lcd_gotoxy(6,1);
         //           lcd_putc('>');
      } // if DATALOOP
      
      //anzeigewert = 0xFF/0x8000*leistung; // 0x8000/0x255 = 0x81
      //anzeigewert = leistung/0x81;
      
      //               cli();
      anzeigewert = leistung /0x18; // /24
      sei();
      
      // if (TEST)
      {
         //   lcd_gotoxy(9,0);
         //   lcd_putint(anzeigewert);
      }
      
      //               webstatus |= (1<<CURRENTSEND);
      currentstatus &= ~(1<<NEWBIT);
      OSZIHI;
      
   } // genuegend Werte
   else
   {
      //lcd_gotoxy(8,1);
      //lcd_puts("    \0");
      
   }
   
   //PORTD &= ~(1<<ECHOPIN);
   impulszeit=0;
   currentstatus &= ~(1<<IMPULSBIT);
   //            OSZIHI;
}