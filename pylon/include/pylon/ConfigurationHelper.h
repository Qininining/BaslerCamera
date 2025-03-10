//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2024 Basler AG
//  http://www.baslerweb.com
//  Author:  Arne Wischmann
//-----------------------------------------------------------------------------
/*!
\file
\brief Helper functions for different camera configuration classes.

       These helper functions are provided as a header-only file.
       The code can be copied and modified for creating your own configuration classes.
*/

#ifndef INCLUDED_CONFIGURATIONHELPER_H_6526739
#define INCLUDED_CONFIGURATIONHELPER_H_6526739


#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */
#include <pylon/ParameterIncludes.h>

namespace Pylon
{

    /** \addtogroup Pylon_InstantCameraApiGeneric
    * @{
    */

    /*!
    \class  CConfigurationHelper
    \brief Helper functions for different camera configuration classes.
    */
    class CConfigurationHelper
    {
    public:

        /*!
        \brief DisableAllTriggers disables all trigger types that can be
        turned off.
        */
        static void DisableAllTriggers( GenApi::INodeMap& nodemap )
        {

            //Disable all trigger types.
            {
                // Get required enumerations.
                CEnumParameter triggerSelector( nodemap, "TriggerSelector" );
                CEnumParameter triggerMode( nodemap, "TriggerMode" );

                if (triggerSelector.IsWritable())
                {
                    // Get all settable enumeration entries of trigger selector.
                    GenApi::StringList_t triggerSelectorEntries;
                    triggerSelector.GetSettableValues( triggerSelectorEntries );

                    const String_t originalSelectorValue = triggerSelector.GetValue();

                    // Turn trigger mode off for all trigger selector entries.
                    for (GenApi::StringList_t::const_iterator it = triggerSelectorEntries.begin(); it != triggerSelectorEntries.end(); ++it)
                    {
                       // Set trigger mode to off if the trigger is available.
                       triggerSelector.SetValue( *it );
                       triggerMode.TrySetValue( "Off" );
                    }

                    // reset to original value
                    triggerSelector.SetValue( originalSelectorValue );
                }
            }
        }

        /*!
        \brief DisableCompression disables all compressions modes that can be
               turned off.
        */
        static void DisableCompression( GenApi::INodeMap& nodemap )
        {
            //Disable compression mode (if supported by the camera).
            CEnumParameter compressionMode( nodemap, "ImageCompressionMode" );

            compressionMode.TrySetValue( "Off" );
        }

        /*!
        \brief DisableGenDC disables GenDC streaming when available.
        */
        static void DisableGenDC( GenApi::INodeMap& nodemap )
        {
            CEnumParameter genDCMode( nodemap, "GenDCStreamingMode" );

            genDCMode.TrySetValue( "Off" );
        }

        /*!
        \brief Select the 'Range' component.

        Some cameras, such as Basler blaze and stereo ace, provide multiple components.
        Default is component 'Range' with pixel format 'Mono16' respective 'Mono8'.
        */
        static void SelectRangeComponent( GenApi::INodeMap& nodemap )
        {
            CEnumParameter componentSelector = CEnumParameter( nodemap, "ComponentSelector" );
            CBooleanParameter enableComponent = CBooleanParameter( nodemap, "ComponentEnable" );
            CEnumParameter pixelFormat = CEnumParameter( nodemap, "PixelFormat" );

            // if multiple components are supported then enable only the 'range' component.
            if (componentSelector.IsWritable())
            {
                const String_t originalComponentValue = componentSelector.GetValue();

                GenApi::StringList_t componentSelectorEntries;
                componentSelector.GetSettableValues( componentSelectorEntries );

                for (GenApi::StringList_t::iterator it = componentSelectorEntries.begin(), end = componentSelectorEntries.end(); it != end; ++it)
                {
                    const String_t& entry = *it;
                    componentSelector.SetValue( entry );

                    // if Range is selected enable it set pixel format to Mono16 of Mono8
                    // for Basler 3D blaze
                    if (entry.compare( "Range" ) == 0)
                    {
                        enableComponent.TrySetValue( true );
                        if (!pixelFormat.TrySetValue( "Mono16" ))
                        {
                            pixelFormat.TrySetValue( "Mono8" );
                        }
                    }
                    else if(    entry.compare( "Intensity" ) == 0
                            ||  entry.compare( "Disparity" ) == 0
                            ||  entry.compare( "Confidence" ) == 0
                            ||  entry.compare( "Error" ) == 0)
                    {
                       // do not change default values for stereo ace
                    }
                    else
                    {   // disable other components
                        enableComponent.TrySetValue( false );
                    }
                }

                // reset the component selector
                componentSelector.TrySetValue( originalComponentValue );

            }
        }

        static void ProbePacketSize( GenApi::INodeMap& nodemap )
        {
            using namespace GenApi;

            CCommandParameter probePacketSize( nodemap, "ProbePacketSize" );

            probePacketSize.TryExecute(  );
        }
    };

    /**
    * @}
    */
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_CONFIGURATIONHELPER_H_6526739 */
