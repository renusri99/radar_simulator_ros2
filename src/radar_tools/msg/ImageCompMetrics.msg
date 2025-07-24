
# info of first image
string topic1
builtin_interfaces/Time stamp1
string frame1

# info of second image
string topic2
builtin_interfaces/Time stamp2
string frame2

# skimage.metrics.structural_similarity
float64 ssi
# skimage.metrics.peak_signal_noise_ratio
float64 psnr
# skimage.metrics.normalized_mutual_information
float64 nmi
# skimage.metrics.variation_of_information
float64[2] voi

# sklearn.metrics.mutual_info_score
float64 mis
