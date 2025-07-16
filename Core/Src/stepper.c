#include "stepper.h"
#include "dwt.h"			//< Библиотека таймера DWT для задержки между фронтами сигнала для пина STEP

/** Функция инициализации шагового мотора
 *
 * 	Функция определяет пины шагового мотора и инициализирует начальные данные мотора -
 * 	позицию, направление, статус включения, глобальное определение поведения пинов DIR, EN
 */
void stepperInit(STEPPER_StructDef* stepper, STEPPER_PINS_StructDef* pins)
{
	stepper->stepper_pins->GPIOx_step = pins->GPIOx_step;
	stepper->stepper_pins->GPIO_Pin_step = pins->GPIO_Pin_step;

	stepper->stepper_pins->GPIOx_dir = pins->GPIOx_dir;
	stepper->stepper_pins->GPIO_Pin_dir = pins->GPIO_Pin_dir;

	stepper->stepper_pins->GPIOx_en = pins->GPIOx_en;
	stepper->stepper_pins->GPIO_Pin_en = pins->GPIO_Pin_en;

	stepper->pos = 0;
	stepper->dir = 1;
	stepper->en = OFF;

	stepper->_globEn = false;
	stepper->_globDir = false;
}

/** Функция выполнения шага мотором
 */
void step(STEPPER_StructDef* stepper)
{
	stepper->pos += stepper->dir;
	setStepperDir(stepper, stepper->dir);

	setPin(stepper->stepper_pins->GPIOx_step, stepper->stepper_pins->GPIO_Pin_step, PIN_SET);
	DWT_usDelay(STEPPER_STEP_TIME);
	setPin(stepper->stepper_pins->GPIOx_step, stepper->stepper_pins->GPIO_Pin_step, PIN_RESET);
}

/** Задать направление вращения мотора в соответствии с величиной _globDir
 * 	если _globDir = false, то GPIO_PIN_SET -> dir = 1,
 * 	PIN_RESET -> dir = -1,
 * 	если _globDir = true, то GPIO_PIN_SET -> dir = -1,
 * 	PIN_RESET -> dir = 1,
 * 	dir = 1 - движение мотора по часовой стрелки
 * 	dir = -1 - движение моторапротив часовой стрелки
 * 	Направление вращения определяется со стороны задней части мотора,
 */
void setStepperDir(STEPPER_StructDef* stepper, int8_t dir)
{
	if(stepper->_globDir == false)
	{
		if(dir == 1)
		{
			stepper->dir = 1;
			setPin(stepper->stepper_pins->GPIOx_dir, stepper->stepper_pins->GPIO_Pin_dir, PIN_SET);
		}
		else
		{
			stepper->dir = -1;
			setPin(stepper->stepper_pins->GPIOx_dir, stepper->stepper_pins->GPIO_Pin_dir, PIN_RESET);
		}
	}
	else if(dir == 1)
	{
		stepper->dir = 1;
		setPin(stepper->stepper_pins->GPIOx_dir, stepper->stepper_pins->GPIO_Pin_dir, PIN_RESET);
	}
		else
		{
			stepper->dir = -1;
			setPin(stepper->stepper_pins->GPIOx_dir, stepper->stepper_pins->GPIO_Pin_dir, PIN_SET);
		}
}

/** Включение мотора
 */
void enableStepper(STEPPER_StructDef* stepper)
{
	stepper->en = ON;

	if(stepper->_globEn == false)
	{
		setPin(stepper->stepper_pins->GPIOx_en, stepper->stepper_pins->GPIO_Pin_en, PIN_SET);
	}
	else setPin(stepper->stepper_pins->GPIOx_en, stepper->stepper_pins->GPIO_Pin_en, PIN_RESET);
}

/** Выключение мотора
 */
void disableStepper(STEPPER_StructDef* stepper)
{
	stepper->en = OFF;

	if(stepper->_globEn == false)
	{
		setPin(stepper->stepper_pins->GPIOx_en, stepper->stepper_pins->GPIO_Pin_en, PIN_RESET);
	}
	else setPin(stepper->stepper_pins->GPIOx_en, stepper->stepper_pins->GPIO_Pin_en, PIN_SET);
}

/** Инвертировать поведение пина EN
 * 	_globEn определяет поведение пина EN
 * 	если _globEn = false, то PIN_SET -> en = ON,
 * 	PIN_RESET -> en = OFF,
 * 	если _globEn = true, то PIN_SET -> en = OFF,
 * 	PIN_RESET -> en = ON,
 */
void invertStepperPinEn(STEPPER_StructDef* stepper)
{
	stepper->_globEn = !stepper->_globEn;
}

/** Инвертировать поведение пина DIR
 * 	_globDir определяет поведение пина DIR
 * 	если _globDir = false, то GPIO_PIN_SET -> dir = 1,
 * 	PIN_RESET -> dir = -1,
 * 	если _globDir = true, то GPIO_PIN_SET -> dir = -1,
 * 	PIN_RESET -> dir = 1,
 * 	dir = 1 - движение мотора по часовой стрелки
 * 	dir = -1 - движение моторапротив часовой стрелки
 * 	Направление вращения определяется со стороны задней части мотора,
 */
void invertStepperPinDir(STEPPER_StructDef* stepper)
{
	stepper->_globDir = !stepper->_globDir;
}
