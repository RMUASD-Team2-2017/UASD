-- phpMyAdmin SQL Dump
-- version 3.5.8.1
-- http://www.phpmyadmin.net
--
-- Host: techgen.dk.mysql:3306
-- Generation Time: Sep 23, 2017 at 12:18 PM
-- Server version: 10.1.26-MariaDB-1~xenial
-- PHP Version: 5.4.45-0+deb7u11

SET SQL_MODE="NO_AUTO_VALUE_ON_ZERO";
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;

--
-- Database: `techgen_dk`
--

-- --------------------------------------------------------

--
-- Table structure for table `AED_drone_list`
--

CREATE TABLE IF NOT EXISTS `AED_drone_list` (
  `int_id` int(10) NOT NULL AUTO_INCREMENT,
  `id` int(10) NOT NULL,
  `name` varchar(100) NOT NULL,
  `deployments` int(10) NOT NULL,
  `loc_lat` float(8,5) NOT NULL,
  `loc_lng` float(8,5) NOT NULL,
  `state` varchar(100) NOT NULL,
  `cur_lat` float(8,5) NOT NULL,
  `cur_lng` float(8,5) NOT NULL,
  `target_lat` float(8,5) NOT NULL,
  `target_lng` float(8,5) NOT NULL,
  `path` text NOT NULL,
  `weather` text NOT NULL,
  `comment` text NOT NULL,
  `extra1` text NOT NULL,
  `extra2` text NOT NULL,
  `extra3` text NOT NULL,
  `extra4` text NOT NULL,
  `extra5` text NOT NULL,
  PRIMARY KEY (`int_id`)
) ENGINE=InnoDB  DEFAULT CHARSET=utf8 AUTO_INCREMENT=3 ;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
