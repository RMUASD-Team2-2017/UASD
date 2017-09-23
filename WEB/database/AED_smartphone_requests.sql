-- phpMyAdmin SQL Dump
-- version 3.5.8.1
-- http://www.phpmyadmin.net
--
-- Host: techgen.dk.mysql:3306
-- Generation Time: Sep 23, 2017 at 12:19 PM
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
-- Table structure for table `AED_smartphone_requests`
--

CREATE TABLE IF NOT EXISTS `AED_smartphone_requests` (
  `int_id` int(10) NOT NULL AUTO_INCREMENT,
  `id` varchar(32) NOT NULL,
  `loc_lat` float(22,18) NOT NULL,
  `loc_lng` float(22,18) NOT NULL,
  `req_time` bigint(15) NOT NULL,
  `timestamp` bigint(15) NOT NULL,
  `loc_accuracy` float NOT NULL,
  `altitude` float NOT NULL,
  `altitude_accuracy` int(11) NOT NULL,
  PRIMARY KEY (`int_id`)
) ENGINE=InnoDB  DEFAULT CHARSET=utf8 AUTO_INCREMENT=389 ;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
