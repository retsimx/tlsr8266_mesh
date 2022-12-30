/********************************************************************************************************
 * @file     DataStorage.java 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
package com.telink.bluetooth.light.model;

import java.util.List;

public interface DataStorage<E> {

	void add(E e);

	void add(E e, int location);

	void add(List<E> e);

	boolean contains(E e);

	boolean contains(String attributeName, Object attributeValue);

	E get(String attributeName, Object attributeValue);

	List<E> get();

	E get(int location);

	void remove(int location);

	void remove(E e);

	int size();

	boolean isEmpty();

	void clear();
}
