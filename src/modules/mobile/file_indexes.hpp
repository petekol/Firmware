#pragma once

#include <limits.h>

#include "activity/activity_files.hpp"

#include "protocol.h"


/*
 * Index structure.
 */

enum FileCatalog : uint8_t
{
	INVALID,
	ACTIVITY,
	PUBLIC,
	AUTH_DEPRICATED, // Two be removed! Useless 100% sure after 2015-11-01.
};

FileCatalog
catalog(file_index_t index)
{
	if (index == 1) { return FileCatalog::PUBLIC; }
	if ((index >> 8) == 0x000001) { return FileCatalog::AUTH_DEPRICATED; }
	if ((index >> 16) == 0x0001) { return FileCatalog::ACTIVITY; }
	return FileCatalog::INVALID;
}

using filename_buf_t = char[PATH_MAX];

/*
 * Activity files.
 */

void
parse_activity_index(file_index_t index, uint8_t & activity, uint8_t & attribute)
{
	activity = (index >> 8) & 0xFF;
	attribute = index & 0xFF;
}

bool
get_activity_filename(file_index_t index, filename_buf_t & pathname)
{
	using namespace AirDog;
	uint8_t activity, attribute;
	parse_activity_index(index, activity, attribute);
	return Activity::Files::get_path(activity, attribute, pathname);
}

bool
is_activity_index_valid(file_index_t index)
{
	using namespace AirDog;
	uint8_t activity, attribute;
	parse_activity_index(index, activity, attribute);
	return Activity::Files::has_valid_id(activity, attribute);
}

bool
is_activity_file_valid(const char tmp_path[], file_index_t index)
{
	using namespace AirDog;
	uint8_t activity, attribute;
	parse_activity_index(index, activity, attribute);
	return Activity::Files::has_valid_content(activity, attribute, tmp_path);
}


/*
 * General files.
 */

bool
get_filename(file_index_t index, filename_buf_t & name)
{
	FileCatalog c = catalog(index);
	bool ok;
	switch (c)
	{
	case FileCatalog::ACTIVITY:
		ok = get_activity_filename(index, name);
		break;
	case FileCatalog::AUTH_DEPRICATED:
		ok = true;
		switch (index)
		{
		case 0x100: case 0x101:
			strncpy(name, "/etc/mobile-empty.dat", sizeof name);
			break;
		default:
			ok = false;
		}
		break;
	case FileCatalog::PUBLIC:
		strncpy(name, "/fs/microsd/mobile/public.dat", sizeof name);
		ok = index == 1;
		break;
	default:
		ok = false;
	}
	if (ok) { dbg("File index 0x%08x name '%s'.\n", index, name); }
	else
	{
		*name = '\0';
		dbg("No name for file index 0x%08x.\n", index);
	}
	return ok;
}

bool
is_file_index_valid(file_index_t index)
{
	FileCatalog c = catalog(index);
	return ((c == FileCatalog::ACTIVITY) and is_activity_index_valid(index))
		or (c != FileCatalog::INVALID);
}

bool
is_file_writable(file_index_t index)
{
	FileCatalog c = catalog(index);
	return c == FileCatalog::ACTIVITY or c == FileCatalog::PUBLIC;
}

bool
is_file_content_valid(const char tmp_path[], file_index_t index)
{
	FileCatalog c = catalog(index);
	switch (c)
	{
	case FileCatalog::ACTIVITY:
		return is_activity_file_valid(tmp_path, index);
	case FileCatalog::PUBLIC:
		return true;
	default:
		return false;
	}
}
