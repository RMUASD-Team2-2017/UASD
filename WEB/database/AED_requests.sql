-- phpMyAdmin SQL Dump
-- version 3.5.8.1
-- http://www.phpmyadmin.net
--
-- Host: techgen.dk.mysql:3306
-- Generation Time: Oct 16, 2017 at 12:08 PM
-- Server version: 10.1.28-MariaDB-1~xenial
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
-- Table structure for table `AED_requests`
--

CREATE TABLE IF NOT EXISTS `AED_requests` (
  `int_id` int(10) NOT NULL AUTO_INCREMENT,
  `drone_id` int(10) NOT NULL,
  `request_id` varchar(48) NOT NULL,
  `time` bigint(15) NOT NULL,
  `completed` bigint(15) NOT NULL,
  `eta` bigint(15) NOT NULL,
  `approved` bigint(15) NOT NULL,
  PRIMARY KEY (`int_id`)
) ENGINE=InnoDB  DEFAULT CHARSET=utf8 AUTO_INCREMENT=167 ;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
