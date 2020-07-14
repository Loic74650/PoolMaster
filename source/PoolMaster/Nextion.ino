/*
  NEXTION TFT related code, based on EasyNextion library by Seithan / Athanasios Seitanis (https://github.com/Seithan/EasyNextionLibrary)
  (c) Loic74 <loic74650@gmail.com> 2018-2020
*/

int CurrentPage = 0;
char HourBuffer[8];
uint8_t debounceCount = 2;
uint8_t debounceM = 0;
uint8_t debounceF = 0;
uint8_t debounceH = 0;
uint8_t debounceR0 = 0;
uint8_t debounceR1 = 0;
uint8_t debounceR2 = 0;

//Structure holding the measurement values to display on the Nextion display
struct TFTStruct
{
  float pH, Orp, pHSP, OrpSP, WT, WTSP, AT, PSI;
  uint8_t FSta, FSto, pHTkFill, OrpTkFill;
  boolean Mode, NetW, Filt, Heat, R0, R1, R2, pHUTErr, ChlUTErr, PSIErr, pHTLErr, ChlTLErr;
  unsigned long pHPpRT, OrpPpRT;
  String FW;
} TFTStruc =
{ //default values.
  7.30, 730, 7.4, 740, 27.11, 28.5, 25.5, 0.80,
  8, 20, 100, 100,
  1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0,
  ""
};


//function to update TFT display
//it updates the TFTStruct variables, the global variables of the TFT + the widgets of the active page
//call this function at least every second to ensure fluid display
void UpdateTFT()
{
  myNex.NextionListen();

  sprintf(HourBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  myNex.writeStr("page0.vaTime.txt", HourBuffer);

  if (Firmw != TFTStruc.FW)
  {
    TFTStruc.FW = F("MC fw: v ");
    TFTStruc.FW += Firmw;
    myNex.writeStr(F("page0.vaMCFW.txt"), TFTStruc.FW);
  }

  if (MQTTConnection != TFTStruc.NetW)
  {
    TFTStruc.NetW = MQTTConnection;
    myNex.writeNum(F("page1.vabNetW.val"), TFTStruc.NetW);

    if (CurrentPage == 0)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p0NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p0NetW.pic=6"));
      }
    }
    else if (CurrentPage == 1)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p1NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p1NetW.pic=6"));
      }
    }
    else if (CurrentPage == 2)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p2NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p2NetW.pic=6"));
      }
    }
    else if (CurrentPage == 3)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
  }

  if (storage.PhValue != TFTStruc.pH)
  {
    TFTStruc.pH = storage.PhValue;
    myNex.writeStr(F("page0.vapH.txt"), String(TFTStruc.pH, 2));
    if (CurrentPage == 0)  myNex.writeStr(F("pH.txt"), String(TFTStruc.pH, 2));
  }
  if (storage.OrpValue != TFTStruc.Orp)
  {
    TFTStruc.Orp = storage.OrpValue;
    myNex.writeStr(F("page0.vaOrp.txt"), String(TFTStruc.Orp, 2));
    if (CurrentPage == 0)  myNex.writeStr(F("Orp.txt"), String(TFTStruc.Orp, 2));
  }
  if (storage.Ph_SetPoint != TFTStruc.pHSP)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    String temp = "(" + String(TFTStruc.pHSP, 1) + ")";
    myNex.writeStr(F("page0.vapHSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHSP.txt"), temp);
  }
  if (storage.Orp_SetPoint != TFTStruc.OrpSP)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    String temp = "(" + String((int)TFTStruc.OrpSP) + ")";
    myNex.writeStr(F("page0.vaOrpSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpSP.txt"), temp);
  }
  if (storage.TempValue != TFTStruc.WT)
  {
    TFTStruc.WT = storage.TempValue;
    String temp = String(TFTStruc.WT, 2) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaWT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("W.txt"), temp);
  }
  if (storage.TempExternal != TFTStruc.AT)
  {
    TFTStruc.AT = storage.TempExternal;
    String temp = String(TFTStruc.AT, 2) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaAT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("A.txt"), temp);
  }
  if (storage.PSIValue != TFTStruc.PSI)
  {
    TFTStruc.PSI = storage.PSIValue;
    String temp = String(TFTStruc.PSI, 2) + F("b");
    myNex.writeStr(F("page0.vaPSI.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("P.txt"), temp);
  }

  if ((storage.FiltrationStop != TFTStruc.FSto) || (storage.FiltrationStart != TFTStruc.FSta))
  {
    TFTStruc.FSto = storage.FiltrationStop;
    TFTStruc.FSta = storage.FiltrationStart;
    String temp = String(TFTStruc.FSta) + F("/") + String(TFTStruc.FSto) + F("h");
    if (CurrentPage == 0)  myNex.writeStr(F("page0.p0StaSto.txt"), temp);
    else if (CurrentPage == 1)  myNex.writeStr(F("page1.p1StaSto.txt"), temp);
    else if (CurrentPage == 2)  myNex.writeStr(F("page2.p2StaSto.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("page3.p3StaSto.txt"), temp);
  }

  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || ((int)(storage.ChlFill - ChlPump.GetTankUsage()) != TFTStruc.OrpTkFill))
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;
    TFTStruc.OrpTkFill = (int)(storage.ChlFill - ChlPump.GetTankUsage());

    String temp = String(TFTStruc.OrpTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.OrpPpRT / 1000 / 60), 1) + F("min");
    myNex.writeStr(F("page0.vaOrpTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpTk.txt"), temp);
  }

  if ((PhPump.UpTime != TFTStruc.pHPpRT) || ((int)(storage.AcidFill - PhPump.GetTankUsage()) != TFTStruc.pHTkFill))
  {
    TFTStruc.pHPpRT = PhPump.UpTime;
    TFTStruc.pHTkFill = (int)(storage.AcidFill - PhPump.GetTankUsage());

    String temp = String(TFTStruc.pHTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.pHPpRT / 1000 / 60), 1) + F("min");
    myNex.writeStr(F("page0.vapHTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHTk.txt"), temp);
  }

  if (storage.AutoMode != TFTStruc.Mode)
  {
    if ((debounceM == 0) || (debounceM > debounceCount))
    {
      debounceM = 0;
      TFTStruc.Mode = storage.AutoMode;
      String temp = TFTStruc.Mode ? F("AUTO") : F("MANU");
      myNex.writeNum(F("page1.vabMode.val"), storage.AutoMode);
      if (CurrentPage == 0)
      {
        myNex.writeStr(F("p0Mode.txt"), temp);
      }
      else if (CurrentPage == 1)
      {
        myNex.writeStr(F("p1Mode.txt"), temp);
        myNex.writeStr(F("t1Mode.txt"), temp);
        if (storage.AutoMode == 1)
          myNex.writeStr(F("bMode.pic=8"));
        else
          myNex.writeStr(F("bMode.pic=7"));
      }
      else if (CurrentPage == 2)
      {
        myNex.writeStr(F("p2Mode.txt"), temp);
      }
      else if (CurrentPage == 3)
      {
        myNex.writeStr(F("p3Mode.txt"), temp);
      }
    }
    else
      debounceM++;
  }

  if (FiltrationPump.IsRunning() != TFTStruc.Filt)
  {
    if ((debounceF == 0) || (debounceF > debounceCount))
    {
      debounceF = 0;
      TFTStruc.Filt = FiltrationPump.IsRunning();
      myNex.writeNum(F("page1.vabFilt.val"), TFTStruc.Filt);
      if (CurrentPage == 1)
      {
        if (TFTStruc.Filt == 1)
          myNex.writeStr(F("bFilt.pic=8"));
        else
          myNex.writeStr(F("bFilt.pic=7"));
      }
    }
    else
      debounceF++;
  }

  if (storage.WaterHeat != TFTStruc.Heat)
  {
    if ((debounceH == 0) || (debounceH > debounceCount))
    {
      debounceH = 0;
      TFTStruc.Heat = storage.WaterHeat;
      myNex.writeNum(F("page1.vabHeat.val"), TFTStruc.Heat);
      if (CurrentPage == 1)
      {
        if (TFTStruc.Heat == 1)
          myNex.writeStr(F("bHeat.pic=8"));
        else
          myNex.writeStr(F("bHeat.pic=7"));
      }
    }
    else
      debounceH++;
  }

  if (digitalRead(RELAY_R1) != TFTStruc.R0)
  {
    if ((debounceR0 == 0) || (debounceR0 > debounceCount))
    {
      debounceR0 = 0;
      TFTStruc.R0 = digitalRead(RELAY_R1);
      myNex.writeNum(F("page1.vabR0.val"), TFTStruc.R0);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R0 == 1)
          myNex.writeStr(F("bR0.pic=8"));
        else
          myNex.writeStr(F("bR0.pic=7"));
      }
    }
    else
      debounceR0++;
  }

  if (digitalRead(RELAY_R2) != TFTStruc.R1)
  {
    if ((debounceR1 == 0) || (debounceR1 > debounceCount))
    {
      debounceR1 = 0;
      TFTStruc.R1 = digitalRead(RELAY_R2);
      myNex.writeNum(F("page1.vabR1.val"), TFTStruc.R1);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R1 == 1)
          myNex.writeStr(F("bR1.pic=8"));
        else
          myNex.writeStr(F("bR1.pic=7"));
      }
    }
    else
      debounceR1++;
  }

  if (digitalRead(RELAY_R6) != TFTStruc.R2)
  {
    if ((debounceR2 == 0) || (debounceR2 > debounceCount))
    {
      debounceR2 = 0;
      TFTStruc.R2 = digitalRead(RELAY_R6);

      myNex.writeNum(F("page1.vabR2.val"), TFTStruc.R2);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R2 == 1)
          myNex.writeStr(F("bR2.pic=8"));
        else
          myNex.writeStr(F("bR2.pic=7"));
      }
    }
    else
      debounceR2++;
  }

  if (ChlPump.TankLevel() != TFTStruc.ChlTLErr)
  {
    TFTStruc.ChlTLErr = ChlPump.TankLevel();
    if (!TFTStruc.ChlTLErr)
    {
      myNex.writeStr(F("page0.vaChlLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlLevel.val=0"));
  }

  if (PhPump.TankLevel() != TFTStruc.pHTLErr)
  {
    TFTStruc.pHTLErr = PhPump.TankLevel();
    if (!TFTStruc.pHTLErr)
    {
      myNex.writeStr(F("page0.vaAcidLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaAcidLevel.val=0"));
  }

  if (PSIError != TFTStruc.PSIErr)
  {
    TFTStruc.PSIErr = PSIError;
    if (TFTStruc.PSIErr)
    {
      myNex.writeStr(F("page0.vaPSIErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaPSIErr.val=0"));
  }

  if (ChlPump.UpTimeError != TFTStruc.ChlUTErr)
  {
    TFTStruc.ChlUTErr = ChlPump.UpTimeError;
    if (TFTStruc.ChlUTErr)
    {
      myNex.writeStr(F("page0.vaChlUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlUTErr.val=0"));
  }

  if (PhPump.UpTimeError != TFTStruc.pHUTErr)
  {
    TFTStruc.pHUTErr = PhPump.UpTimeError;
    if (TFTStruc.pHUTErr)
    {
      myNex.writeStr(F("page0.vapHUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vapHUTErr.val=0"));
  }

  //update time at top of displayed page
  switch (CurrentPage)
  {
    case 0: {
        myNex.writeStr(F("p0Time.txt"), HourBuffer);
        break;
      }
    case 1: {
        myNex.writeStr(F("p1Time.txt"), HourBuffer);
        break;
      }
    case 2: {
        myNex.writeStr(F("p2Time.txt"), HourBuffer);
        break;
      }
    case 3: {
        myNex.writeStr(F("p3Time.txt"), HourBuffer);
        break;
      }
  }
}

//reset TFT at start of controller
void ResetTFT()
{
  myNex.writeStr(F("rest"));
}

//Page 0 has finished loading
//printh 23 02 54 01
void trigger1()
{
  CurrentPage = 0;
  DEBUG_PRINT(F("Nextion p0"));
}

//Page 1 has finished loading
//printh 23 02 54 02
void trigger2()
{
  CurrentPage = 1;
  DEBUG_PRINT(F("Nextion p1"));
}

//Page 2 has finished loading
//printh 23 02 54 03
void trigger3()
{
  CurrentPage = 2;
  DEBUG_PRINT(F("Nextion p2"));
}

//Page 3 has finished loading
////printh 23 02 54 04
void trigger4()
{
  CurrentPage = 3;
  DEBUG_PRINT(F("Nextion p3"));
}

//MODE button was toggled
//printh 23 02 54 05
void trigger5()
{
  TFTStruc.Mode = (boolean)myNex.readNumber(F("vabMode.val"));
  debounceM = 1;
  DEBUG_PRINT(F("MODE button"));
  if (TFTStruc.Mode)
  {
    String Cmd = F("{\"Mode\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Mode\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//FILT button was toggled
//printh 23 02 54 06
void trigger6()
{
  TFTStruc.Filt = (boolean)myNex.readNumber(F("vabFilt.val"));
  debounceF = 1;
  DEBUG_PRINT(F("FILT button"));
  if (TFTStruc.Filt)
  {
    String Cmd = F("{\"FiltPump\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"FiltPump\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//HEAT button was toggled
//printh 23 02 54 07
void trigger7()
{
  TFTStruc.Heat = (boolean)myNex.readNumber(F("vabHeat.val"));
  debounceH = 1;
  DEBUG_PRINT(F("HEAT button"));
  if (TFTStruc.Heat)
  {
    String Cmd = F("{\"Heat\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Heat\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 0 button was toggled
//printh 23 02 54 08
void trigger8()
{
  TFTStruc.R0 = (boolean)myNex.readNumber(F("vabR0.val"));
  debounceR0 = 1;
  DEBUG_PRINT(F("Relay 0 button"));
  if (TFTStruc.R0)
  {
    String Cmd = F("{\"Relay\":[1,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[1,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 1 button was toggled
//printh 23 02 54 09
void trigger9()
{
  TFTStruc.R1 = (boolean)myNex.readNumber(F("vabR1.val"));
  debounceR1 = 1;
  DEBUG_PRINT(F("Relay 1 button"));
  if (TFTStruc.R1)
  {
    String Cmd = F("{\"Relay\":[2,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[2,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 2 button was toggled
//printh 23 02 54 0A
void trigger10()
{
  TFTStruc.R2 = (boolean)myNex.readNumber(F("vabR2.val"));
  debounceR2 = 1;
  DEBUG_PRINT(F("Relay 2 button"));
  if (TFTStruc.R2)
  {
    String Cmd = F("{\"Relay\":[6,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[6,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Probe calibration completed or new pH, Orp or Water Temp setpoints or New tank
//printh 23 02 54 0B
void trigger11()
{
  DEBUG_PRINT("Calibration complete or new pH, Orp or Water Temp setpoints or new tank event");
  String Cmd = myNex.readStr(F("pageCalibs.vaCommand.txt"));
  queueIn.enqueue(Cmd);
  DEBUG_PRINT(Cmd);
}

//Clear Errors button pressed
//printh 23 02 54 0C
void trigger12()
{
  DEBUG_PRINT(F("Clear errors event"));
  String Cmd = F("{\"Clear\":1}");
  queueIn.enqueue(Cmd);
}
