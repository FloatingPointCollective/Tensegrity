/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for Arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "DW1000Time.h"

DW1000Time::DW1000Time() {
	memset(_timestamp, 0, LEN_STAMP);
}

DW1000Time::DW1000Time(byte data[]) {
	setFromBytes(data);
}

DW1000Time::DW1000Time(const DW1000Time& copy) {
	copy.getAsBytes(_timestamp);
}

DW1000Time::DW1000Time(unsigned long value, float factorUs) {
	float tsValue = value * factorUs;
	tsValue = fmod(tsValue, TIME_OVERFLOW);
	setFromFloat(tsValue);
}

DW1000Time::~DW1000Time() {}

void DW1000Time::setFromFloat(float time) {
	int i = 0;
	byte val = 0;
	memset(_timestamp, 0, LEN_STAMP);
	time *= TIME_RES_INV;
	while(i < LEN_STAMP && time >= 1) {
		_timestamp[i] = ((byte)fmod(time, 256.0f) & 0xFF);
		time = floor(time / 256.0f);
		i++;
	} 
}

void DW1000Time::setFromBytes(byte data[]) {
	memcpy(_timestamp, data, LEN_STAMP);
}

float DW1000Time::getAsFloat() const {
	float tsValue = (_timestamp[0] & 0xFF) * TIME_RES;
	tsValue += ((_timestamp[1] & 0xFF) * 0.004006410256f);
	tsValue += ((_timestamp[2] & 0xFF) * 1.02564102564f);
	tsValue += ((_timestamp[3] & 0xFF) * 262.564102564f);
	tsValue += ((_timestamp[4] & 0xFF) * 67216.4098339f);
	return tsValue;
}

/*uint64_t DW1000Time::getAsInt() const {
	uint64_t tsValue = _timestamp[0];
	tsValue |= ((uint64_t)_timestamp[1] << 8);
	tsValue |= ((uint64_t)_timestamp[2] << 16);
	tsValue |= ((uint64_t)_timestamp[3] << 24);
	tsValue |= ((uint64_t)_timestamp[4] << 32);
	return tsValue * TIME_RES;
}*/

void DW1000Time::getAsBytes(byte data[]) const {
	memcpy(data, _timestamp, LEN_STAMP);
}

DW1000Time& DW1000Time::operator=(const DW1000Time &assign) {
	if(this == &assign) {
		return *this;
	}
	assign.getAsBytes(_timestamp);
	return *this;
}

DW1000Time& DW1000Time::operator+=(const DW1000Time &add) {
	addTimestampBytes(_timestamp, _timestamp, add._timestamp);
	return *this;
}

const DW1000Time DW1000Time::operator+(const DW1000Time &add) const {
    return DW1000Time(*this) += add;
}

DW1000Time& DW1000Time::operator-=(const DW1000Time &sub) {
	subtractTimestampBytes(_timestamp, _timestamp, sub._timestamp);
	return *this;
}

const DW1000Time DW1000Time::operator-(const DW1000Time &sub) const {
	return DW1000Time(*this) -= sub;
}

boolean DW1000Time::operator==(const DW1000Time &cmp) const {
	return memcmp(_timestamp, cmp._timestamp, LEN_STAMP) == 0;
}

boolean DW1000Time::operator!=(const DW1000Time &cmp) const {
	return !(*this == cmp);
}

void DW1000Time::addTimestampBytes(byte r[], byte a[], const byte b[]) {
	unsigned int carry = 0;
	for(int i = 0; i < LEN_STAMP; i++) {
		// add up numbers, take a byte and preserve carry for next round
		carry += (a[i] + b[i]);
		r[i] = (carry & 0xFF);
		carry >>= 8;
	}
	// note: usually one would add up carry to higher-order bytes of result,
	// but we simply ignore and hence obtain a counter wrapped-around one.
}

void DW1000Time::subtractTimestampBytes(byte r[], byte a[], const byte b[]) {
	// get the one's complement
	byte negB[LEN_STAMP];
	for(int i = 0; i < LEN_STAMP; i++) {
		negB[i] = ~b[i];
	}
	// get the two's complement
	const byte one[] = {0x01, 0x00, 0x00, 0x00, 0x00};
	addTimestampBytes(negB, negB, one);
	// add up numbers
	addTimestampBytes(r, a, negB);
}
