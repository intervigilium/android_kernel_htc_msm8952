/****************************************************************************
 * Company:         Fairchild Semiconductor
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * G. Noblesmith
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the “Company”)
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/

#ifndef __VDM_BITFIELD_TRANSLATORS_H__
#define __VDM_BITFIELD_TRANSLATORS_H__

#include "../platform.h"

	UnstructuredVdmHeader 	getUnstructuredVdmHeader(UINT32 in);	
	StructuredVdmHeader 	getStructuredVdmHeader(UINT32 in);		
	IdHeader 				getIdHeader(UINT32 in);					
	VdmType 				getVdmTypeOf(UINT32 in);				

	UINT32 	getBitsForUnstructuredVdmHeader(UnstructuredVdmHeader in);	
	UINT32 	getBitsForStructuredVdmHeader(StructuredVdmHeader in);		
	UINT32 	getBitsForIdHeader(IdHeader in);							

	CertStatVdo 			getCertStatVdo(UINT32 in);
	ProductVdo 				getProductVdo(UINT32 in);
	CableVdo 				getCableVdo(UINT32 in);
	AmaVdo 					getAmaVdo(UINT32 in);

 	UINT32 	getBitsForProductVdo(ProductVdo in);	
	UINT32 	getBitsForCertStatVdo(CertStatVdo in);	
	UINT32	getBitsForCableVdo(CableVdo in);		
	UINT32	getBitsForAmaVdo(AmaVdo in);			

#endif 

