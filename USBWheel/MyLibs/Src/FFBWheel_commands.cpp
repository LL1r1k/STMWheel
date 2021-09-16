#include "FFBWheel.h"
#include "SimDisplayProtocol.h"

RGB defaultRGB[] = {
		{0,0,0},
		{128,0,0},
		{0,128,0},
		{0,0,128}
};

void FFBWheel::executeCommands(std::vector<ParsedCommand> commands){
	std::string reply;
	extern std::vector<CommandHandler*> cmdHandlers;
	for(ParsedCommand cmd : commands){
		if(!executeSysCommand(&cmd,&reply)){
			// Call all command handlers
			for(CommandHandler* handler : cmdHandlers){
				if(handler->hasCommands())
					if(handler->command(&cmd,&reply))
						break; // Stop after this class if finished flag is returned
			}

		}
		if(!reply.empty() && reply.back()!='\n'){
			reply+='\n';
		}
	}
	if(reply.length()>0){
		CDC_Transmit_FS(reply.c_str(), reply.length());
	}
}

bool FFBWheel::executeSysCommand(ParsedCommand* cmd,std::string* reply){
	bool flag = true;
	if(cmd->cmd == "help"){
		*reply += parser.helpstring;
		*reply += "Commands:format (Erase flash)";
		flag = false; // Continue to user commands
	}else if(cmd->cmd == "format"){
		if(cmd->type == CMDtype::set && cmd->val==1){
			HAL_FLASH_Unlock();
			EE_Format();
			HAL_FLASH_Lock();
		}else{
			*reply += "format=1 will ERASE ALL stored variables. Be careful!";
		}
	}else{
		flag = false;
	}
	return flag;
}
volatile const SimDisplayPacket* Oldtelemetry;
bool FFBWheel::command(ParsedCommand* cmd,std::string* reply){
	bool flag = true;
	// ------------ General commands ----------------
	if(cmd->cmd == "save"){
		needSave = true;
		*reply+="OK";
	}else if(cmd->cmd == "zeroenc"){
		if(cmd->type == CMDtype::get){
			this->enc->setPos(0);

			*reply += "OK";
		}
	}else if(cmd->cmd == "maxPower"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(conf.maxpower);
		}else if(cmd->type == CMDtype::set){
			this->conf.maxpower = cmd->val;
			float effect_margin_scaler = ((float)conf.totalGain/255.0);
			this->torqueScaler = ((float)this->conf.maxpower / (float)0x7fff) * effect_margin_scaler;
			*reply += "OK";
		}
	}else if(cmd->cmd == "degrees"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(this->conf.degreesOfRotation);
		}else if(cmd->type == CMDtype::set){
			this->conf.degreesOfRotation = cmd->val;
			enc->degree = conf.degreesOfRotation;
			*reply += "OK";
		}
	}else if(cmd->cmd == "axismask"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(this->conf.axes);
		}else if(cmd->type == CMDtype::set){
			this->conf.axes = cmd->val;
			*reply += "OK";
		}
	}else if(cmd->cmd == "ppr"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(this->enc->getPpr());
		}else if(cmd->type == CMDtype::set && this->enc != nullptr){
			this->conf.encoderPPR = cmd->val;
			this->enc->setPpr(cmd->val);
			*reply += "OK";
		}else{
			*reply += "Err. Setup enctype first";
		}
	}else if(cmd->cmd == "adcmax"){
			if(cmd->type == CMDtype::get){
				*reply+=std::to_string(this->conf.maxAdcCount);
			}else if(cmd->type == CMDtype::set){
				this->conf.maxAdcCount = cmd->val;
				*reply += "OK";
			}
	}else if(cmd->cmd == "inverted"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.inverted);
				}else if(cmd->type == CMDtype::set){
					this->conf.inverted = (uint8_t)cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "constantGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.constantGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.constantGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "springGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.springGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.springGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "damperGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.damperGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.damperGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "inertiaGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.inertiaGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.inertiaGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "frictionGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.frictionGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.frictionGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "endstopGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.endstop_gain);
				}else if(cmd->type == CMDtype::set){
					this->conf.endstop_gain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "totalGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.totalGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.totalGain = cmd->val;
					float effect_margin_scaler = ((float)conf.totalGain/255.0);
					this->torqueScaler = ((float)this->conf.maxpower / (float)0x7fff) * effect_margin_scaler;
					*reply += "OK";
				}
	}else if(cmd->cmd == "minPower"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.minForce);
				}else if(cmd->type == CMDtype::set){
					this->conf.minForce = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "wheelNUM"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.wheelNUM);
				}else if(cmd->type == CMDtype::set){
					this->conf.wheelNUM = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "pos"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(this->enc->getPos());
		}else if(cmd->type == CMDtype::set && this->enc != nullptr){
			this->enc->setPos(cmd->val);
			*reply += "OK";
		}else{
			*reply += "Err. Setup enctype first";
		}
	}else if(cmd->cmd == "default"){
			if(cmd->type == CMDtype::get){
				FFBWheelConfig defConf;
				this->conf = defConf;
				*reply += "OK";
			}
	}else if(cmd->cmd == "all"){
		if(cmd->type == CMDtype::get){
			uint8_t* buf8 = (uint8_t*)&conf;
			uint8_t len = sizeof(FFBWheelConfig);
			for(uint8_t i = 0; i < len ;i++)
				*reply += (char)buf8[i];
		}
	}else if(cmd->cmd == "hidrate" && cmd->type == CMDtype::get){
		if(ffb->hid_out_period != 0){
			*reply+=std::to_string(1000/ffb->hid_out_period);
		}else{
			*reply+="0";
		}
	}else if(cmd->cmd == "led"){
		if(cmd->type == CMDtype::set){
			RGB rgb_array[16] = {0};
			for(uint8_t i = 0;i<LED_COUNT;i++)
			{
				uint8_t tmp = (cmd->val >> (2*i)) & 0b11;
				if(tmp >=0 && tmp <=3)
					rgb_array[i] = defaultRGB[tmp];
			}
			setup_ws2812(rgb_array, LED_COUNT);
			*reply+="OK";
		}
	}else if(cmd->type == CMDtype::dash){
		flag = true;
		Oldtelemetry = (SimDisplayPacket*)(cmd->cmd.c_str());
		uint16_t rgb_array = 0;
		if (Oldtelemetry->status == SDP_STATUS_OFF)
			setup_rpm_ws2812(rgb_array);
		else if (Oldtelemetry->status == SDP_STATUS_LIVE)
		{
			//*reply+= std::to_string(telemetry->rpm) + " : " + std::to_string(telemetry->optrpm) + " : " + std::to_string(telemetry->shftrpm);
			uint16_t maxrpm = Oldtelemetry->optrpm/90*100;
			uint16_t shiftrpm = Oldtelemetry->optrpm/95*100;
			uint16_t optrpm = Oldtelemetry->optrpm;
			if(Oldtelemetry->rpm <= maxrpm)
			{
				if(Oldtelemetry->rpm < optrpm)
					rgb_array = std::max(0, ((Oldtelemetry->rpm - 1500) * 8 / (optrpm- 1500)));
				else if(Oldtelemetry->rpm < shiftrpm)
					rgb_array = 8 + (Oldtelemetry->rpm - optrpm)*4/(shiftrpm - optrpm);
				else
					rgb_array = 12 + (Oldtelemetry->rpm - shiftrpm)*4/((shiftrpm*100/95) - shiftrpm);
				setup_rpm_ws2812(rgb_array);
			}
		}
	}else if(cmd->cmd == "help"){
		flag = false;
		*reply += ""
				", save, zeroenc, maxPower, degrees, axismask, ppr, adcmax, inverted, constantGain, rampGain, squareGain, sinGain, triangleGain, sawToothDownGain, sawToothUpGain, springGain, damperGain, inertiaGain, frictionGain, endstopGain, totalGain, maxVelosity, maxAcceleration, maxPositionChange, minPower, pos, hidrate, led, all, default, help\n"; // TODO
	}else if(cmd->type == CMDtype::simhub){
		uint16_t size = cmd->cmd.length();
		char* pstr = (char*)cmd->cmd.c_str();
		uint16_t count = 0;
		uint16_t i = 0;
		uint16_t idx = 0;
		for(;i<size;i++)
			if(*(pstr+i) == ':')
			{
				count = atoi(pstr+i+1);
				i++;
				break;
			}
		for(;i<size;i++)
			if(*(pstr+i) == ':')
			{
				int16_t tmp = atoi(pstr+i+1);
				i2cBuffer[idx++] = tmp;
			}
		i2cSize = count * 2;
	}else{
		//flag = false;
	}


	return flag;
}

