/*
 * Copyright (c) 2009 Andrew Nguyen
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

#ifndef __WINE_T2EMBAPI_H
#define __WINE_T2EMBAPI_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHARSET_UNICODE   1
#define CHARSET_DEFAULT   1
#define CHARSET_SYMBOL    2
#define CHARSET_GLYPHIDX  3

#define LICENSE_INSTALLABLE   0x0000
#define LICENSE_DEFAULT       0x0000
#define LICENSE_NOEMBEDDING   0x0002
#define LICENSE_PREVIEWPRINT  0x0004
#define LICENSE_EDITABLE      0x0008

#define TTLOAD_PRIVATE  0x0001

/* Possible return values. */
#define E_NONE                              __MSABI_LONG(0x0000)
#define E_API_NOTIMPL                       __MSABI_LONG(0x0001)

typedef ULONG (WINAPIV * READEMBEDPROC)(void*,void*,ULONG);
typedef ULONG (WINAPIV * WRITEEMBEDPROC)(void*,void*,ULONG);

typedef struct
{
    unsigned short usStructSize;
    unsigned short usRefStrSize;
    unsigned short *pusRefStr;
} TTLOADINFO;

typedef struct
{
    unsigned short usStructSize;
    unsigned short usRootStrSize;
    unsigned short *pusRootStr;
} TTEMBEDINFO;

LONG WINAPI TTLoadEmbeddedFont(HANDLE*,ULONG,ULONG*,ULONG,ULONG*,READEMBEDPROC,
                               LPVOID,LPWSTR,LPSTR,TTLOADINFO*);
LONG WINAPI TTDeleteEmbeddedFont(HANDLE,ULONG,ULONG*);

/* embedding privileges */
#define EMBED_PREVIEWPRINT  1
#define EMBED_EDITABLE      2
#define EMBED_INSTALLABLE   3
#define EMBED_NOEMBEDDING   4

LONG WINAPI TTGetEmbeddingType(HDC, ULONG*);

#ifdef __cplusplus
}
#endif

#endif
