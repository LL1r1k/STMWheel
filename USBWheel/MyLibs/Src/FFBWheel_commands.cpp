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
volatile const SimDisplayPacket* telemetry;
bool FFBWheel::command(ParsedCommand* cmd,std::string* reply){
	bool flag = true;
	// ------------ General commands ----------------
	if(cmd->cmd == "save"){
		this->saveFlash();
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
			*reply += "OK";
		}
	}else if(cmd->cmd == "degrees"){
		if(cmd->type == CMDtype::get){
			*reply+=std::to_string(this->conf.degreesOfRotation);
		}else if(cmd->type == CMDtype::set){
			this->conf.degreesOfRotation = cmd->val;
			enc->maxAngle = conf.degreesOfRotation;
			enc->maxValue = (float)enc->maxAngle / 2 / 360 * enc->ppr;
			enc->minValue = -enc->maxValue;
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
					this->conf.inverted = cmd->val;
					*reply += "OK";
				}

	}else if(cmd->cmd == "constantGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.constantGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.constantGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "rampGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.rampGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.rampGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "squareGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.squareGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.squareGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "sinGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.sinGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.sinGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "triangleGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.triangleGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.triangleGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "sawToothDownGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.sawToothDownGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.sawToothDownGain = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "sawToothUpGain"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.sawToothUpGain);
				}else if(cmd->type == CMDtype::set){
					this->conf.sawToothUpGain = cmd->val;
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
					*reply += "OK";
				}
	}else if(cmd->cmd == "maxVelosity"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.maxVelosity);
				}else if(cmd->type == CMDtype::set){
					this->conf.maxVelosity = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "maxAcceleration"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.maxAcceleration);
				}else if(cmd->type == CMDtype::set){
					this->conf.maxAcceleration = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "maxPositionChange"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.maxPositionChange);
				}else if(cmd->type == CMDtype::set){
					this->conf.maxPositionChange = cmd->val;
					*reply += "OK";
				}
	}else if(cmd->cmd == "minPower"){
				if(cmd->type == CMDtype::get){
					*reply+=std::to_string(this->conf.minForce);
				}else if(cmd->type == CMDtype::set){
					this->conf.minForce = cmd->val;
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
		telemetry = (SimDisplayPacket*)(cmd->cmd.c_str());
		uint8_t rgb_array = 0;
		if (telemetry->status == SDP_STATUS_OFF)
			setup_rpm_ws2812(rgb_array);
		else if (telemetry->status == SDP_STATUS_LIVE && telemetry->rpm <= (telemetry->shftrpm*100/95))
		{
			if(telemetry->rpm < telemetry->optrpm)
				rgb_array = std::max(0, telemetry->rpm - 1500) * 8 / (telemetry->optrpm - 1500);
			else if(telemetry->rpm < telemetry->shftrpm)
				rgb_array = 8 + (telemetry->rpm - telemetry->optrpm)*4/(telemetry->shftrpm - telemetry->optrpm);
			else
				rgb_array = 12 + (telemetry->rpm - telemetry->shftrpm)*4/((telemetry->shftrpm*100/95) - telemetry->shftrpm);
			setup_rpm_ws2812(rgb_array);
		}
	}else if(cmd->cmd == "help"){
		flag = false;
		*reply += ""
				", save, zeroenc, maxPower, degrees, axismask, ppr, adcmax, inverted, constantGain, rampGain, squareGain, sinGain, triangleGain, sawToothDownGain, sawToothUpGain, springGain, damperGain, inertiaGain, frictionGain, endstopGain, totalGain, maxVelosity, maxAcceleration, maxPositionChange, minPower, pos, hidrate, led, all, help\n"; // TODO
	}else{
		flag = false;
	}


	return flag;
}

