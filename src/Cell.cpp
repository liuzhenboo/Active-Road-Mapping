#include <iostream>
#include <Cell.h>

Cell::Cell()
{
}
Cell::Cell(uint8_t class_state, float current_height)
{
	state = 3;
	seen_times = 0;
	obs_seentimes = 0;
	ground_seentimes = 0;
	obs_height = 2;
	ground_height = 2;
	Update(class_state, current_height);
}
Cell::~Cell()
{
}

uint8_t Cell::Update(uint8_t class_state, float current_height)
{
	uint8_t fusion_id = class_state;
	// 更新格子状态
	if (class_state == 1)
	{
		ground_height = (1.0 * current_height + 0.0 * ground_height);
		ground_seentimes++;
	}
	else if (class_state == 2)
	{
		obs_height = (1.0 * current_height + 0.0 * obs_height);
		obs_seentimes++;
	}

	if (obs_seentimes >= ground_seentimes)
	{
		fusion_id = 2;
	}
	else
	{
		fusion_id = 1;
	}
	// else
	// {
	// 	std::cout << "发现状态为：３" << std::endl;
	// 	fusion_id = 3;
	// }
	state = fusion_id;
	return fusion_id;
}
float Cell::GetZ(int id)
{
	if (state == 1)
	{
		return ground_height;
	}
	else if (state == 2)
	{
		return obs_height;
	}
	std::cout << "debug" << std::endl;
	return obs_height;
}
uint8_t Cell::GetState()
{
	return state;
}
