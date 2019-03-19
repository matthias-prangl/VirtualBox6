/*
 * Copyright (C) 2010 Maarten Lankhorst for CodeWeavers
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 */

/*
 * Oracle LGPL Disclaimer: For the avoidance of doubt, except that if any license choice
 * other than GPL or LGPL is available it will apply instead, Oracle elects to use only
 * the Lesser General Public License version 2.1 (LGPLv2) at this time for any software where
 * a choice of LGPL license versions is made available with the language indicating
 * that LGPLv2 or any later version may be used, or where a choice of which version
 * of the LGPL is applied is otherwise unspecified.
 */

#ifndef __AMAUDIO__
#define __AMAUDIO__

#include <mmsystem.h>
#include <dsound.h>

#undef INTERFACE
#define INTERFACE IAMDirectSound

DECLARE_INTERFACE_(IAMDirectSound,IUnknown)
{
    /*** IUnknown methods ***/
    STDMETHOD_(HRESULT,QueryInterface)(THIS_ REFIID riid, void** ppvObject) PURE;
    STDMETHOD_(ULONG,AddRef)(THIS) PURE;
    STDMETHOD_(ULONG,Release)(THIS) PURE;

    /*** IAMDirectSound methods ***/
    STDMETHOD(GetDirectSoundInterface)(THIS_ IDirectSound **ds) PURE;
    STDMETHOD(GetPrimaryBufferInterface)(THIS_ IDirectSoundBuffer **buf) PURE;
    STDMETHOD(GetSecondaryBufferInterface)(THIS_ IDirectSoundBuffer **buf) PURE;
    STDMETHOD(ReleaseDirectSoundInterface)(THIS_ IDirectSound *ds) PURE;
    STDMETHOD(ReleasePrimaryBufferInterface)(THIS_ IDirectSoundBuffer *buf) PURE;
    STDMETHOD(ReleaseSecondaryBufferInterface)(THIS_ IDirectSoundBuffer *buf) PURE;
    STDMETHOD(SetFocusWindow)(THIS_ HWND hwnd, BOOL bgaudible) PURE;
    STDMETHOD(GetFocusWindow)(THIS_ HWND *hwnd, BOOL *bgaudible) PURE;
};
#undef INTERFACE

#endif
