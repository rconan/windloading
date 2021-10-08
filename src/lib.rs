//! This module is used to create the structure that applied wind forces and moments on the telescope FEM
//!
//! There are 7 wind load sources applied to the following elements of the telescope structure:
//!  - the C-Rings
//!  - the GIR
//!  - the M1 cells
//!  - the M1 segments
//!  - the trusses
//!  - the M2 segments
//!  - the top-end

use dosio::{
    io::{jar, Tags},
    DOSIOSError, Dos, IOTags, IO,
};
use serde;
use serde::{Deserialize, Serialize};
use std::{fmt, fs::File, io, io::BufReader, path::Path};

#[derive(Debug)]
pub enum WindLoadsError {
    Len,
    Empty,
    FileNotFound(io::Error),
    PickleRead(serde_pickle::Error),
    Outputs,
    Inputs,
}
impl fmt::Display for WindLoadsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Len => f.write_str("couldn't get the number of sample in the time series"),
            Self::Empty => f.write_str("no data available"),
            Self::FileNotFound(e) => write!(f, "wind loads data file not found: {}", e),
            Self::PickleRead(e) => write!(f, "cannot read wind loads data file: {}", e),
            Self::Outputs => f.write_str(""),
            Self::Inputs => f.write_str("WindLoading takes no inputs"),
        }
    }
}
impl From<std::io::Error> for WindLoadsError {
    fn from(e: std::io::Error) -> Self {
        Self::FileNotFound(e)
    }
}
impl From<serde_pickle::Error> for WindLoadsError {
    fn from(e: serde_pickle::Error) -> Self {
        Self::PickleRead(e)
    }
}
impl std::error::Error for WindLoadsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::FileNotFound(source) => Some(source),
            Self::PickleRead(source) => Some(source),
            _ => None,
        }
    }
}

type Result<T> = std::result::Result<T, WindLoadsError>;
type Outputs = Option<std::vec::IntoIter<Vec<f64>>>;

macro_rules! loads {
    ($($name:expr, $variant:ident),+) => {
        /// Wind loads forces and moments
        ///
        /// A time vector containing vectors of forces and moments
        #[derive(Serialize, Deserialize, Debug,Clone)]
        pub enum Loads {
            $(#[serde(rename = $name)]
              $variant(Vec<Vec<f64>>)),+
        }
        impl Loads {
            /// Returns the number of samples in the time series
            pub fn len(&self) -> usize {
                match self {
                    $(Loads::$variant(io) => io.len()),+
                }
            }
            /// Return the loads
            pub fn io(self) -> Vec<Vec<f64>> {
                match self {
                    $(Loads::$variant(io) => io),+
                }
            }
            pub fn decimate(&mut self, decimation_rate: usize) {
                match self {
                    $(Loads::$variant(io) => {
                        let decimated: Vec<_> = io.iter()
                            .step_by(decimation_rate)
                            .cloned()
                            .collect();
                        *io = decimated;
                    }),+
                }
            }
            pub fn range(&mut self, min_index: usize, max_index: usize) {
                match self {
                    $(Loads::$variant(io) => {
                        let ranged: Vec<_> = io.iter()
                            .skip(min_index)
                            .take(max_index-min_index)
                            .cloned()
                            .collect();
                        *io = ranged;
                    }),+
                }
            }
        }
	pub fn wind_loads_name() -> Vec<String> {
	    vec![$($name.to_string()),+]
	}
    };
}
loads!(
    "OSS_TopEnd_6F",
    OSSTopEnd6F,
    "OSS_Truss_6F",
    OSSTruss6F,
    "OSS_GIR_6F",
    OSSGIR6F,
    "OSS_CRING_6F",
    OSSCRING6F,
    "OSS_Cell_lcl_6F",
    OSSCellLcl6F,
    "OSS_M1_lcl_6F",
    OSSM1Lcl6F,
    "MC_M2_lcl_force_6F",
    MCM2LclForce6F,
    "OSS_mirrorCovers_6F",
    OSSMirrorCovers6F
);

pub trait MatchWindLoads {
    fn data(&self, wind_loads: &Loads) -> Option<std::vec::IntoIter<Vec<f64>>>;
    fn ndata(&self, wind_loads: &Loads, n: usize) -> Option<std::vec::IntoIter<Vec<f64>>>;
}
macro_rules! io_match_wind_loads {
    ($($variant:ident),+) => {
        impl<T: std::fmt::Debug> MatchWindLoads for IO<T> {
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator over the first `n` elements
            fn data(&self, wind_loads: &Loads) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, Loads::$variant(v)) => Some(v.clone().into_iter()),)+
                        (_, _) => None,
                }
            }
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator
            fn ndata(&self, wind_loads: &Loads, n: usize) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, Loads::$variant(v)) => Some(v[..n].to_owned().into_iter()),)+
                        (_, _) => None,
                }
            }
        }
    };
}
io_match_wind_loads!(
    OSSTopEnd6F,
    OSSTruss6F,
    OSSGIR6F,
    OSSCRING6F,
    OSSCellLcl6F,
    OSSM1Lcl6F,
    MCM2LclForce6F,
    OSSMirrorCovers6F
);

/// Wind loads builder
///
/// This structure is used to read the forces and moments time series from a data file and to create the [`WindLoading`] structure
#[derive(Default, Deserialize, Serialize)]
pub struct WindLoads {
    /// forces and moments time series
    #[serde(rename = "outputs")]
    pub loads: Vec<Option<Loads>>,
    /// time vector
    pub time: Vec<f64>,
    #[serde(skip)]
    n_sample: Option<usize>,
    #[serde(skip)]
    tagged_loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
}

impl WindLoads {
    /// Reads the wind loads from a pickle file
    pub fn from_pickle<P: AsRef<Path>>(path: P) -> Result<Self> {
        let f = File::open(path)?;
        let r = BufReader::with_capacity(1_000_000_000, f);
        serde_pickle::from_reader(r).map_err(WindLoadsError::PickleRead)
        //        Ok(pkl::from_value(v)?)
    }
    /// Returns the number of samples in the time series
    fn len(&self) -> Result<usize> {
        self.loads
            .iter()
            .find_map(|x| x.as_ref().and_then(|x| Some(x.len())))
            .ok_or(WindLoadsError::Len)
    }
    pub fn range(mut self, t_min: f64, t_max: f64) -> Self {
        let min_index = self.time.iter().position(|t| *t >= t_min).unwrap_or(0);
        let max_index = self
            .time
            .iter()
            .position(|t| *t >= t_max)
            .unwrap_or(self.time.len());
        self.loads
            .iter_mut()
            .filter_map(|x| x.as_mut())
            .for_each(|x| {
                x.range(min_index, max_index);
            });
        self
    }
    pub fn decimate(mut self, decimation_rate: usize) -> Self {
        self.loads
            .iter_mut()
            .filter_map(|x| x.as_mut())
            .for_each(|x| {
                x.decimate(decimation_rate);
            });
        self
    }
    fn tagged_load(&self, io: &Tags) -> Result<Outputs> {
        match &self.n_sample {
            Some(n) => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.ndata(x, *n)))
                .map_or(Err(WindLoadsError::Empty), |x| Ok(Some(x))),
            None => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.data(x)))
                .map_or(Err(WindLoadsError::Empty), |x| Ok(Some(x))),
        }
    }
    /// Set the number of time sample
    pub fn n_sample(self, n_sample: usize) -> Result<Self> {
        assert!(n_sample > 0, "n_sample must be greater than 0");
        let n = self.len()?;
        assert!(
            n_sample <= n,
            "n_sample cannot be greater than the number of sample ({})",
            n
        );
        Ok(Self {
            n_sample: Some(if n_sample <= n { n_sample } else { n }),
            ..self
        })
    }
    /// Selects loads on the truss
    pub fn truss(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSTruss6F {
            data: self.tagged_load(&jar::OSSTruss6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the top-end
    pub fn topend(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSTopEnd6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::io())?,
        });
        Ok(self)
    }
    pub fn m2_asm_topend(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2TE6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the C-ring
    pub fn cring(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSCRING6F {
            data: self.tagged_load(&jar::OSSCRING6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the GIR
    pub fn gir(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSGIR6F {
            data: self.tagged_load(&jar::OSSGIR6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 cells
    pub fn m1_cell(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSCellLcl6F {
            data: self.tagged_load(&jar::OSSCellLcl6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 segments
    pub fn m1_segments(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSM1Lcl6F {
            data: self.tagged_load(&jar::OSSM1Lcl6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 mirror covers
    pub fn m1_covers(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSMirrorCovers6F {
            data: self.tagged_load(&jar::OSSMirrorCovers6F::io())?,
        });
        Ok(self)
    }
    /// Selects loads on the M2 segments
    pub fn m2_segments(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2LclForce6F {
            data: self.tagged_load(&jar::MCM2LclForce6F::io())?,
        });
        Ok(self)
    }
    /// Associates a FEM input with the loads on the M2 segments
    pub fn m2_segments_into(mut self, fem: IO<()>) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2LclForce6F {
            data: self.tagged_load(&fem)?,
        });
        Ok(self)
    }
    pub fn m2_asm_reference_bodies(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2RB6F {
            data: self.tagged_load(&jar::MCM2LclForce6F::io())?,
        });
        Ok(self)
    }
    /// Selects all loads
    pub fn select_all(self) -> Result<Self> {
        self.topend()?
            .m2_segments()?
            .truss()?
            .m1_segments()?
            .m1_cell()?
            .gir()?
            .cring()
    }
    /// Selects all loads in the ASM configuration
    pub fn select_all_with_asm(self) -> Result<Self> {
        self.m2_asm_topend()?
            .m2_asm_reference_bodies()?
            .truss()?
            .m1_segments()?
            .m1_cell()?
            .gir()?
            .cring()
    }
    /// Builds a wind loading source object
    pub fn build(self) -> Result<WindLoading> {
        Ok(WindLoading {
            n_sample: self.n_sample.unwrap_or(self.len()?),
            loads: self.tagged_loads,
        })
    }
}

/// Wind loading sources
///
/// This structure contains the time series of wind forces and moments.
/// The time series implement the [`Iterator`] trait and the [`outputs`](crate::wind_loads::WindLoading::outputs) method step through the iterator
#[derive(Default)]
pub struct WindLoading {
    pub loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
    pub n_sample: usize,
}

/// Wind loading interface
impl IOTags for WindLoading {
    fn outputs_tags(&self) -> Vec<Tags> {
        self.loads.iter().map(|x| x.into()).collect()
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        unimplemented!("WindLoading takes no inputs")
    }
}
impl Dos for WindLoading {
    type Input = ();
    type Output = Vec<f64>;
    fn inputs(
        &mut self,
        _: Option<Vec<IO<Self::Input>>>,
    ) -> std::result::Result<&mut Self, DOSIOSError> {
        Err(DOSIOSError::Inputs((WindLoadsError::Inputs).into()))
    }
    fn outputs(&mut self) -> Option<Vec<IO<Self::Output>>> {
        self.loads
            .iter_mut()
            .map(|x| -> Option<IO<Vec<f64>>> { x.into() })
            .collect()
    }
}
