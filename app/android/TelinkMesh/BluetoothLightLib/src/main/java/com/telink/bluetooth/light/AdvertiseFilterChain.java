/********************************************************************************************************
 * @file     AdvertiseFilterChain.java 
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
/*
 * Copyright (C) 2015 The Telink Bluetooth Light Project
 *
 */
package com.telink.bluetooth.light;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * 广播过滤器链
 */
public final class AdvertiseFilterChain {

    private static final AdvertiseFilterChain DEFAULT_CHAIN = new AdvertiseFilterChain("Telink default filter chain");

    static {
        DEFAULT_CHAIN.add(DefaultAdvertiseDataFilter.create());
    }

    private String name;
    private List<AdvertiseDataFilter> mFilters;

    private AdvertiseFilterChain(String name) {
        this.name = name;
        this.mFilters = new ArrayList<>();
    }

    public static AdvertiseFilterChain getDefault() {
        return DEFAULT_CHAIN;
    }

    public String getName() {
        return name;
    }

    public AdvertiseFilterChain add(AdvertiseDataFilter filter) {
        synchronized (this) {
            this.mFilters.add(filter);
        }
        return this;
    }

    public AdvertiseFilterChain remove(AdvertiseDataFilter filter) {
        synchronized (this) {
            if (this.mFilters.contains(filter)) {
                this.mFilters.remove(filter);
            }
        }
        return this;
    }

    public AdvertiseFilterChain removeAll() {
        synchronized (this) {
            this.mFilters.clear();
        }
        return this;
    }

    public Iterator<AdvertiseDataFilter> iterator() {
        synchronized (this) {
            return this.mFilters.iterator();
        }
    }
}
