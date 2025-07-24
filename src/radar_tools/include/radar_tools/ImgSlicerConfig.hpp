// radar_tools/ImgSlicerConfig.hpp
#pragma once

namespace radar_tools
{
    struct ImgSlicerConfig
    {
        int col_min = 0;    // Cut Col Min
        int col_max = 0;    // Cut Col Max
        int row_min = 0;    // Cut Row Min
        int row_max = 0;    // Cut Row Max

        static constexpr int MIN_VALUE = 0;
        static constexpr int MAX_VALUE = 100000;
    };
}


    
    
